/**
 * @file main.c
 * @brief GNC Test Bench Telemetry System - Main Application
 * @author Flight Systems Team
 * @date 2025-10-06
 * @version 1.2.0
 * 
 * @details
 * This is the main telemetry collection system for the GNC (Guidance, Navigation, 
 * and Control) test bench. It interfaces with multiple sensors including GPS, 
 * magnetometer, RTC, and IMU to provide comprehensive flight data.
 * 
 * The system uses a non-blocking round-robin scheduler to collect data from all
 * sensors without blocking any single device. GPS data is recieved via UART 
 * interrupts with double-buffering to prevent data loss.
 * 
 * @warning This code is flight-critical. Any modifications must be thoroughly
 * tested before deployment.
 * 
 * @note GPS time sync requires a valid 3D fix before proceeding to main loop
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"

#include "QMC5883L/qmc5883l.h"
#include "DS3231/ds3231.h"
#include "MPU6050/mpu6050.h"
#include "NEOM8N/neom8n.h"

/*============================================================================*/
/* CONFIGURATION PARAMETERS                                                   */
/*============================================================================*/

/** @defgroup Config Configuration Parameters
 *  @{
 */

/// I2C peripheral instance for sensor communication
#define I2C_PORT i2c0

/// I2C SDA pin number (GPIO 1)
#define I2C_SDA 1

/// I2C SCL pin number (GPIO 0)
#define I2C_SCL 0

/// UART TX pin for NEOM8N GPS module
#define NEOM8N_UART_TX_PIN 12

/// UART RX pin for NEOM8N GPS module
#define NEOM8N_UART_RX_PIN 13

/// UART baud rate for GPS communication (9600 is NEOM8N default)
#define NEOM8N_UART_BAUDRATE 9600

/// Maximum size of NMEA sentence buffer
#define NMEA_BUFFER_SIZE 128

/// Number of NMEA sentence types we're intrested in
#define NMEA_NUM_PREFIXES 3

/// Maximum prefix length for sentence matching
#define MAX_PREFIX_LEN 6

/// GPS sync timeout in milliseconds (120 seconds)
#define GPS_SYNC_TIMEOUT_MS 120000

/// Telemetry print interval (every N loops)
#define TELEMETRY_PRINT_INTERVAL 1000

/// Watchdog timeout in milliseconds (8 seconds)
#define WATCHDOG_TIMEOUT_MS 8000

/// Header for the telemetry packet (0xABAB)
#define FIFO_HEADER_UINT16 	0xABAB

/// Uncomment to define DEBUG statements and print it
#define DEBUG

/** @} */ // end of Config group

/*============================================================================*/
/* DATA STRUCTURES                                                            */
/*============================================================================*/

/** @defgroup DataStructures Data Structures
 *  @{
 */

/**
 * @brief Complete telemetry packet structure
 * 
 * This structure holds all sensor data collected in one telemetry cycle.
 * It includes magnetometer, RTC, and GPS data from multiple NMEA sentences.
 * 
 * @note Size of this struct should be minimized for efficient memory usage
 * on the RP2040's limited RAM (264KB)
 */
#pragma pack(push,1)
typedef struct TLM_packet {
    qmc_5883_mag_read_t mag;      ///< Magnetometer readings (X, Y, Z)
    timeframe_rtc_t timeframe;    ///< Real-time clock timestamp
    nmea_gnrmc_t gnrmc;           ///< GPS RMC sentence (position, speed)
    nmea_gngga_t gngga;           ///< GPS GGA sentence (fix quality, altitude)
    nmea_gnvtg_t gnvtg;           ///< GPS VTG sentence (velocity)
} TLM_packet_t;
#pragma pack(pop)

/*
 * @brief telemetry serial from the telemetry packet
 * 
 * This structure takes the TLM Packet and adds a header and footer to ident
 * the first and last of the frame. also packed as tightly as possible
 * 
 * @note Size of this struct should be minimized for efficient memory usage
 * and also be padded to not let leaks in
 */
#pragma pack(push,1)
typedef struct TLM_serial {
	uint16_t header;			///< Header for the serial file
	TLM_packet_t packet;		///< The packet being packed
	uint16_t CRC16;				///< Footer of CRC16 to check its integrity
}TLM_serial_t;
#pragma pack(pop)

/**
 * @brief Device enumeration for round-robin scheduling
 * 
 * Defines the order in which sensors are polled. Each iteration of the
 * telemetry loop reads from one device to maintain non-blocking operation.
 */
typedef enum {
    DEV_GPS = 0,    ///< GPS module (NEOM8N)
    DEV_MAG,        ///< Magnetometer (QMC5883L)
    DEV_RTC,        ///< Real-time clock (DS3231)
    DEV_IMU,        ///< Inertial measurement unit (MPU6050) - TBD
    DEV_COUNT       ///< Total number of devices
} current_device_t;

/** @} */ // end of DataStructures group

/*============================================================================*/
/* NMEA PARSING STATE                                                         */
/*============================================================================*/

/** @defgroup NMEA NMEA Parsing
 *  @{
 */

/// NMEA sentence prefixes we want to capture
static const char *nmea_prefixes[NMEA_NUM_PREFIXES] = {
    "$GNRMC",  ///< Recommended Minimum Navigation Information
    "$GNGGA",  ///< Global Positioning System Fix Data
    "$GNVTG"   ///< Track Made Good and Ground Speed
};

/// Pre-calculated prefix lengths for efficency
static const uint8_t prefix_lengths[NMEA_NUM_PREFIXES] = {6, 6, 6};

/// Double buffer for NMEA sentence reception (prevents data loss)
static uint8_t nmea_buffers[2][NMEA_BUFFER_SIZE];

/// Index of buffer currently being written by ISR
static volatile uint8_t write_buffer_idx = 0;

/// Current write position in active buffer
static volatile uint16_t nmea_write_idx = 0;

/// Flag indicating complete sentence is ready for parsing
static volatile bool nmea_data_ready = false;

/// Index of buffer ready for reading (-1 if none)
static volatile int8_t ready_buffer_idx = -1;

/**
 * @brief NMEA parser state machine structure
 * 
 * Maintains state for incremental NMEA sentence matching.
 * Uses per-prefix position tracking for efficent parsing.
 */
static volatile struct {
    int8_t matched_prefix;                    ///< Index of matched prefix (-1 if none)
    uint8_t prefix_pos[NMEA_NUM_PREFIXES];   ///< Current position in each prefix
    bool receiving;                           ///< True when activly receiving sentence
} nmea_state = {-1, {0}, false};

/// Critical section for protecting buffer access between ISR and main loop
static critical_section_t nmea_crit_sec;

/** @} */ // end of NMEA group

/*============================================================================*/
/* HARDWARE INITIALIZATION                                                    */
/*============================================================================*/

/** @defgroup HwInit Hardware Initialization
 *  @{
 */

/**
 * @brief Initialize I2C peripheral for sensor communication
 * 
 * Configures I2C0 at 400kHz (fast mode) with appropriate GPIO pins.
 * Enables internal pull-up resistors on SDA and SCL lines.
 * 
 * @note 400kHz is the maximum reliable speed for the sensor combo
 * @note External pull-ups may be needed for long wire runs
 */
static inline void init_i2c(void) {
    // Initialize I2C at 400kHz for fast communication
    i2c_init(I2C_PORT, 400000);
    
    // Configure GPIO pins for I2C function
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    
    // Enable internal pull-ups (still recomend external 4.7k resistors)
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

/**
 * @brief UART interrupt handler for GPS data reception
 * 
 * This is a time-critical ISR that processes incoming NMEA sentences from
 * the GPS module. It uses a state machine to match sentence prefixes and
 * implements double-buffering to prevent data loss.
 * 
 * The ISR performs the following operations:
 * 1. Reads all available characters from UART FIFO
 * 2. Matches against known NMEA prefixes
 * 3. Buffers complete sentences for main loop processing
 * 4. Handles buffer overflow gracefully
 * 
 * @warning This function runs in interrupt context - keep it fast!
 * @note Processes entire FIFO per interrupt to minimize interrupt overhead
 */
void __isr __time_critical_func(uart0_irq_handler)(void) {
    // Process all available characters in one go (reduces interrupt count)
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);

        if (!nmea_state.receiving) {
            // State: Waiting for sentence start - match prefixes
            bool prefix_started = false;

            // Check each prefix for potential match
            for (int i = 0; i < NMEA_NUM_PREFIXES; i++) {
                uint8_t pos = nmea_state.prefix_pos[i];
                
                if (ch == nmea_prefixes[i][pos]) {
                    // Character matches - advance this prefix
                    nmea_state.prefix_pos[i]++;
                    prefix_started = true;

                    // Check if we've matched the complete prefix
                    if (nmea_state.prefix_pos[i] == prefix_lengths[i]) {
                        // Full match! Start recieving this sentence
                        nmea_state.matched_prefix = i;
                        nmea_state.receiving = true;
                        nmea_write_idx = 0;
                        
                        // Copy prefix to buffer
                        memcpy((void*)nmea_buffers[write_buffer_idx], 
                               nmea_prefixes[i], prefix_lengths[i]);
                        nmea_write_idx = prefix_lengths[i];

                        // Reset all prefix trackers
                        memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
                        break;
                    }
                } else {
                    // Mismatch - reset this prefix (but check if it's a new '$')
                    nmea_state.prefix_pos[i] = (ch == '$') ? 1 : 0;
                    if (ch == '$') prefix_started = true;
                }
            }

            // If no prefix matched, reset all trackers
            if (!prefix_started) {
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }

        } else {
            // State: Receiving sentence data
            if (nmea_write_idx < NMEA_BUFFER_SIZE - 1) {
                nmea_buffers[write_buffer_idx][nmea_write_idx++] = ch;

                // Check for sentence terminator
                if (ch == '\n') {
                    // Sentence complete - null terminate and swap buffers
                    nmea_buffers[write_buffer_idx][nmea_write_idx] = '\0';
                    ready_buffer_idx = write_buffer_idx;
                    write_buffer_idx ^= 1; // Toggle buffer (0<->1)
                    nmea_data_ready = true;
                    nmea_state.receiving = false;
                    nmea_state.matched_prefix = -1;
                }
            } else {
                // Buffer overflow! Reset and wait for next sentence
                // This shouldn't happen with proper NMEA sentences (<82 chars)
                nmea_state.receiving = false;
                nmea_write_idx = 0;
                nmea_state.matched_prefix = -1;
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }
        }
    }
}

/**
 * @brief Initialize UART for GPS communication
 * 
 * Configures UART0 for 9600 baud, 8-N-1 format with interrupts enabled.
 * Sets up the ISR for asynchronous data reception.
 * 
 * @note FIFO threshold is set to trigger interrupt when 4+ bytes available
 * @note Interrupt priority is set to highest to prevent GPS data loss
 */
static inline void init_uart(void) {
    // Initialize UART at GPS baudrate
    uart_init(uart0, NEOM8N_UART_BAUDRATE);
    
    // Configure GPIO pins for UART function
    gpio_set_function(NEOM8N_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NEOM8N_UART_RX_PIN, GPIO_FUNC_UART);
    
    // No hardware flow control needed
    uart_set_hw_flow(uart0, false, false);
    
    // 8 data bits, 1 stop bit, no parity
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    
    // Enable FIFO for better performance
    uart_set_fifo_enabled(uart0, true);

    // Set FIFO interrupt threshold (trigger when ≥4 bytes available)
    // This reduces interrupt frequency while maintaining responsiveness
    hw_write_masked(&uart_get_hw(uart0)->ifls,
                    2 << UART_UARTIFLS_RXIFLSEL_LSB,
                    UART_UARTIFLS_RXIFLSEL_BITS);

    // Setup interrupt handler with highest priority
    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
    irq_set_priority(UART0_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(UART0_IRQ, true);
    
    // Enable RX interrupts only (we don't need TX interrupts)
    uart_set_irq_enables(uart0, true, false);
}

/** @} */ // end of HwInit group

/*============================================================================*/
/* GPS TIME SYNCHRONIZATION                                                   */
/*============================================================================*/

/** @defgroup GPSSync GPS Time Sync
 *  @{
 */

/**
 * @brief Synchronize RTC from GPS when valid fix is aquired
 * 
 * This function waits for the GPS to acquire a valid 3D fix and then
 * synchronizes the onboard RTC with GPS time. It's critical for accurate
 * timestamping throughout the flight.
 * 
 * The function performs the following:
 * 1. Waits for valid GNRMC sentence (status 'A')
 * 2. Waits for valid GNGGA sentence (fix quality > 0)
 * 3. Parses date and time from GPS
 * 4. Programs the DS3231 RTC
 * 5. Sets the time_synced flag
 * 
 * @return true if RTC was sucessfully synced
 * @return false if timeout occured before getting valid fix
 * 
 * @note Timeout is set to 60 seconds (cold start worst case)
 * @note This is a blocking function - only call during initialization!
 * @warning Do not proceed with flight operations if sync fails
 */
bool sync_rtc_from_gps(void) {

    #ifdef DEBUG
        printf("Waiting for GPS fix to sync RTC...\n");
    #endif
    
    nmea_gnrmc_t rmc = {0};
    nmea_gngga_t gga = {0};
    nmea_gnvtg_t vtg = {0};
    
    uint32_t timeout = GPS_SYNC_TIMEOUT_MS;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    bool rmc_valid = false;
    bool gga_valid = false;
    
    // Reset NMEA state to ensure we get fresh data (important for warm starts)
    nmea_data_ready = false;
    ready_buffer_idx = -1;
    
    // Wait for valid GPS fix with timeout
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < timeout) {
        // Feed the watchdog so it doesn't reset during GPS aquisition
        watchdog_update();
        
        // Check for new NMEA data
        if (nmea_data_ready) {
            critical_section_enter_blocking(&nmea_crit_sec);
            
            if (ready_buffer_idx >= 0) {
                // Parse the sentence
                nmea_type_t type = nmea_parse_sentence((char*)nmea_buffers[ready_buffer_idx], 
                                                       &rmc, &gga, &vtg);
                
                // Check what type we got and if it's valid
                if (type == NMEA_TYPE_GNRMC && rmc.status == 'A') {
                    rmc_valid = true;
                    #ifdef DEBUG
                        printf("RMC valid: Date=%s Time=%s\n", rmc.date, rmc.utc_time);
                    #endif
                }
                if (type == NMEA_TYPE_GNGGA && gga.fix_quality > 0) {
                    gga_valid = true;
                    #ifdef DEBUG
                        printf("GGA valid: Fix=%d Sats=%d\n", gga.fix_quality, gga.num_satellites);
                    #endif
                }
                
                ready_buffer_idx = -1;
            }
            
            nmea_data_ready = false;
            critical_section_exit(&nmea_crit_sec);
            
            // If we have both valid sentences, sync the RTC
            if (rmc_valid && gga_valid) {
                timeframe_rtc_t rtc_time = {0};
                
                // Parse date from DDMMYY format
                if (strlen(rmc.date) >= 6) {
                    rtc_time.time.day = (rmc.date[0] - '0') * 10 + (rmc.date[1] - '0');
                    rtc_time.time.month = (rmc.date[2] - '0') * 10 + (rmc.date[3] - '0');
                    rtc_time.time.year = 2000 + (rmc.date[4] - '0') * 10 + (rmc.date[5] - '0');
                }
                
                // Parse time from HHMMSS.ss format
                if (strlen(rmc.utc_time) >= 6) {
                    rtc_time.time.hour = (rmc.utc_time[0] - '0') * 10 + (rmc.utc_time[1] - '0');
                    rtc_time.time.minute = (rmc.utc_time[2] - '0') * 10 + (rmc.utc_time[3] - '0');
                    rtc_time.time.second = (rmc.utc_time[4] - '0') * 10 + (rmc.utc_time[5] - '0');
                }
                
                // Mark as synchronized
                rtc_time.time.is_time_synced = true;
                
                // Write to RTC hardware
                ds3231_set_time(I2C_PORT, &rtc_time);
                
                #ifdef DEBUG

                    // Print success message with details
                    printf("✓ RTC synced successfully!\n");
                    printf("  Date: %04d-%02d-%02d\n", 
                        rtc_time.time.year, rtc_time.time.month, rtc_time.time.day);
                    printf("  Time: %02d:%02d:%02d UTC\n", 
                        rtc_time.time.hour, rtc_time.time.minute, rtc_time.time.second);
                    printf("  GPS Fix Quality: %d\n", gga.fix_quality);
                    printf("  Satellites: %d\n", gga.num_satellites);
                    
                return true;
                #endif
            }
        }
        
        // Small delay to prevent busy-waiting and save power
        sleep_ms(10);
    }
    
    // Timeout occured - GPS didn't get fix
    #ifdef DEBUG
        printf("✗ GPS sync timeout - no fix acquired after %d seconds\n", 
            GPS_SYNC_TIMEOUT_MS / 1000);
        return false;
    #endif
}

/** @} */ // end of GPSSync group

/*============================================================================*/
/* TELEMETRY COLLECTION                                                       */
/*============================================================================*/

/** @defgroup Telemetry Telemetry Collection
 *  @{
 */

/**
 * @brief Non-blocking telemetry packet assembly with round-robin scheduling
 * 
 * This function implements a non-blocking round-robin scheduler that reads
 * from one sensor per call. This prevents any single sensor from blocking
 * the collection of data from other sensors.
 * 
 * The scheduling order is:
 * 1. GPS (NEOM8N) - checks for new NMEA sentences
 * 2. Magnetometer (QMC5883L) - reads if data ready
 * 3. RTC (DS3231) - reads current time
 * 4. IMU  - placeholder for future implementation TODO
 * 
 * @param[in,out] tlm_packet Pointer to telemetry packet to update
 * 
 * @note This function should be called continuosly in the main loop
 * @note Each sensor's data is updated independantly when available
 * @warning tlm_packet must be properly initialized before first call
 */
void telemetry_non_blocking_packet(TLM_packet_t* tlm_packet, current_device_t current_device) 
{
    // Process one device per call
    switch(current_device) {
        case DEV_GPS:
            // Check if new GPS data is available
            if (nmea_data_ready) {
                critical_section_enter_blocking(&nmea_crit_sec);
                
                if (ready_buffer_idx >= 0) {
                    // Parse NMEA sentence (parser modifies string, hence non-const)
                    nmea_parse_sentence((char*)nmea_buffers[ready_buffer_idx], 
                                      &tlm_packet->gnrmc, 
                                      &tlm_packet->gngga, 
                                      &tlm_packet->gnvtg);
                    ready_buffer_idx = -1;
                }
                
                nmea_data_ready = false;
                critical_section_exit(&nmea_crit_sec);
            }
            break;

        case DEV_MAG:
            // Read magnetometer if data is ready (non-blocking)
            qmc5883l_read_mag_drdy(I2C_PORT, &tlm_packet->mag);
            break;

        case DEV_RTC:
            // Read current time from RTC
            ds3231_read_time(I2C_PORT, &tlm_packet->timeframe);
            break;

        case DEV_IMU:
            // TODO: Implement IMU reading once MPU6050 driver is fixed
            // Placeholder for accel/gyro data collection
            break;

        default:
            // Should never get here, but handle gracefully
            break;
    }
}

/*============================================================================*/
/* TELEMETRY Serialisation                                                    */
/*============================================================================*/

/** @defgroup Telemetry Telemetry Serialisation
 *  @{
 */



/**
 * @brief Serialise and add header and footer to the Struct to send to second core for
 * processing
 * 
 * This function adds a padding layer to the section and also a CRC16 check
 * 
 * 
 * @param[in,out] tlm_packet Pointer to telemetry packet to update
 * @param[in,out] tlm_serial Pointer to the final serial packet
 * 
 * @note This function should be called continuosly in the main loop
 * @note This function is initialised and to be received by second core in a 
 * interrupt
 * @warning tlm_serial must be properly initialized before first call
 */

void tlm_serialiser(TLM_packet_t* tlm_packet, TLM_serial_t* tlm_serial)
{
    // Initialize CRC
    uint16_t crc = 0xFFFF;

    // Load the header
    tlm_serial->header = FIFO_HEADER_UINT16;

    // Copy the packet into the FIFO packet struct
    memcpy(&tlm_serial->packet, tlm_packet, sizeof(TLM_packet_t));

    // Calculate CRC16 over the packet bytes
    uint8_t* data = (uint8_t*)&tlm_serial->packet;
    for (size_t i = 0; i < sizeof(TLM_packet_t); i++) 
    {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) 
        {
            if (crc & 0x8000) 
                crc = (crc << 1) ^ 0x1021;
            else 
                crc <<= 1;
        }
    }

    // Store CRC in the FIFO packet struct
    tlm_serial->CRC16 = crc;
}



/** @} */ // end of Telemetry group

/*============================================================================*/
/* MAIN APPLICATION                                                           */
/*============================================================================*/

/** @defgroup Main Main Application
 *  @{
 */

/**
 * @brief Main application entry point
 * 
 * Initializes all hardware peripherals and sensors, performs GPS time sync,
 * then enters the main telemetry collection loop. Includes periodic debug
 * output and watchdog monitoring for system reliability.
 * 
 * @return Should never return (infinite loop)
 * 
 * @note Watchdog will reset system if main loop hangs for >8 seconds
 * @note Debug output is printed every 1000 telemetry cycles
 */
int main(void) {
    // Initialize standard I/O (USB serial)
    stdio_init_all();
    
    // Initialize critical section for NMEA buffer protection
    critical_section_init(&nmea_crit_sec);
    
    // Enable watchdog timer for system reliability
    // Will reset if not fed within timeout period
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    // Initialize hardware peripherals
    init_i2c();
    init_uart();

    // Initialize all sensors
    qmc_5883l_init(I2C_PORT);
    ds3231_init(I2C_PORT);
    mpu6050_init(I2C_PORT);

    #ifdef DEBUG
        printf("\n========== GNC TEST BENCH INITIALIZING ==========\n");
        printf("Software Version: 1.2.0\n");
        printf("Build Date: %s %s\n", __DATE__, __TIME__);
    #endif
    
    // Perform critical GPS time synchronization

    bool rtc_synced = sync_rtc_from_gps();
    #ifdef DEBUG
        if (!rtc_synced) {
            printf("⚠ WARNING: Continuing without GPS time sync!\n");
            printf("⚠ Time-critical operations may be affected\n");

        }
        
        printf("=================================================\n\n");
    #endif
    // Initialize telemetry packet
    TLM_packet_t tlm_packet = {0};
	TLM_serial_t tlm_serial = {0};
    uint32_t loop_count = 0;
	
	/// Current device in round-robin schedule
	static current_device_t current_device = DEV_GPS;

    // Main telemetry collection loop
    while (true) {
        // Feed the watchdog to prevent system reset
        watchdog_update();
        
        // Collect telemetry data from one sensor (non-blocking)
        telemetry_non_blocking_packet(&tlm_packet, current_device);

        current_device = (current_device + 1) % DEV_COUNT;

		tlm_serialiser(&tlm_packet, &tlm_serial);

		multicore_fifo_push_blocking((uint32_t)&tlm_serial);

        #ifdef DEBUG
        // Print debug telemetry periodically
            if (++loop_count % TELEMETRY_PRINT_INTERVAL == 0) {
                printf("\n========== TELEMETRY PACKET #%lu ==========\n", 
                    loop_count / TELEMETRY_PRINT_INTERVAL);
                
                // RTC timestamp
                printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d %s\n",
                    tlm_packet.timeframe.time.year,
                    tlm_packet.timeframe.time.month,
                    tlm_packet.timeframe.time.day,
                    tlm_packet.timeframe.time.hour,
                    tlm_packet.timeframe.time.minute,
                    tlm_packet.timeframe.time.second,
                    tlm_packet.timeframe.time.is_time_synced ? "[SYNCED]" : "[NOT SYNCED]");
                
                // Magnetometer readings in raw counts
                printf("[MAG] X:%d Y:%d Z:%d\n",
                    tlm_packet.mag.mag_x,
                    tlm_packet.mag.mag_y,
                    tlm_packet.mag.mag_z);
                
                // GPS GNRMC - Recomended Minimum Navigation Info
                printf("[GNRMC] Time:%s Date:%s Status:%c Lat:%s%c Lon:%s%c Speed:%.2fkts Course:%.2f°\n",
                    tlm_packet.gnrmc.utc_time,
                    tlm_packet.gnrmc.date,
                    tlm_packet.gnrmc.status,
                    tlm_packet.gnrmc.lat,
                    tlm_packet.gnrmc.ns,
                    tlm_packet.gnrmc.lon,
                    tlm_packet.gnrmc.ew,
                    tlm_packet.gnrmc.speed_knots,
                    tlm_packet.gnrmc.course_deg);
                
                // GPS GNGGA - Fix quality and altitude data
                printf("[GNGGA] Time:%s Lat:%.6f%c Lon:%.6f%c Fix:%d Sats:%d HDOP:%.2f Alt:%.2fm\n",
                    tlm_packet.gngga.utc_time,
                    tlm_packet.gngga.lat,
                    tlm_packet.gngga.ns,
                    tlm_packet.gngga.lon,
                    tlm_packet.gngga.ew,
                    tlm_packet.gngga.fix_quality,
                    tlm_packet.gngga.num_satellites,
                    tlm_packet.gngga.hdop,
                    tlm_packet.gngga.altitude);
                
                // GPS GNVTG - Velocity and course data
                printf("[GNVTG] Course(T):%.2f° Course(M):%.2f° Speed:%.2fkts (%.2fkm/h)\n",
                    tlm_packet.gnvtg.course_true,
                    tlm_packet.gnvtg.course_magnetic,
                    tlm_packet.gnvtg.speed_knots,
                    tlm_packet.gnvtg.speed_kmh);
                
                printf("==========================================\n\n");
        }
        #endif
        
        // Optional: Add small delay to prevent CPU saturation
        // Uncomment if you need to reduce power consumption
        // tight_loop_contents();
    }

    // Should never reach here
    return 0;
}

/** @} */ // end of Main group

/**
 * @page DesignNotes Design Notes and Implementation Details
 * 
 * @section arch System Architecture
 * 
 * The telemetry system is designed around a non-blocking round-robin scheduler
 * that ensures fair access to all sensors without blocking. This is critical
 * for real-time data collection where any sensor could potentially have delays.
 * 
 * @section buffering Double Buffering Strategy
 * 
 * GPS data uses a double-buffer approach where one buffer is being written by
 * the ISR while the other is being read by the main loop. This prevents data
 * loss and race conditions without needing complex synchronization.
 * 
 * @section timing Timing Considerations
 * 
 * - GPS updates at 1Hz (once per second)
 * - Magnetometer can update up to 200Hz (we poll when data ready)
 * - RTC is read once per round-robin cycle
 * - Total loop time depends on sensor response times
 * 
 * @section memory Memory Usage
 * 
 * The system uses approximately:
 * - 256 bytes for NMEA buffers (128 x 2)
 * - ~200 bytes for telemetry packet structure
 * - Stack usage varies but should stay under 2KB
 * 
 * @section safety Safety Features
 * 
 * - Watchdog timer prevents system hangs
 * - Buffer overflow protection in NMEA parser
 * - GPS time sync validation before flight operations
 * - Critical sections protect shared data
 * 
 * @section future Future Improvements
 * 
 * - Implement DMA for UART to reduce interrupt overhead
 * - Add data logging to SD card
 * - Implement Kalman filtering for sensor fusion
 * - Add telemetry downlink via radio
 * - Complete IMU integration
 * 
 * @author Vedarsh Reddy Muniratnam
 * @date 2025-10-06
 */