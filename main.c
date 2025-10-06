/**
 * @file main.c
 * @brief GNC Test Bench Telemetry System - Main Application
 * @author Flight Systems Team
 * @date 2025-10-06
 * @version 1.2.1
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

#define I2C_PORT i2c0
#define I2C_SDA 1
#define I2C_SCL 0
#define NEOM8N_UART_TX_PIN 12
#define NEOM8N_UART_RX_PIN 13
#define NEOM8N_UART_BAUDRATE 9600
#define NMEA_BUFFER_SIZE 128
#define NMEA_NUM_PREFIXES 3
#define MAX_PREFIX_LEN 6
#define GPS_SYNC_TIMEOUT_MS 120000
#define TELEMETRY_PRINT_INTERVAL 1000
#define WATCHDOG_TIMEOUT_MS 8000
#define FIFO_HEADER_UINT16 0xABAB
#define TX_POOL_SIZE 8

// Uncomment for debug output
// #define DEBUG

/*============================================================================*/
/* DATA STRUCTURES                                                            */
/*============================================================================*/

#pragma pack(push,1)
typedef struct TLM_packet {
    qmc_5883_mag_read_t mag;
    timeframe_rtc_t timeframe;
    nmea_gnrmc_t gnrmc;
    nmea_gngga_t gngga;
    nmea_gnvtg_t gnvtg;
} TLM_packet_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct TLM_serial {
    uint16_t header;
    TLM_packet_t packet;
    uint16_t CRC16;
} TLM_serial_t;
#pragma pack(pop)

typedef enum {
    DEV_GPS = 0,
    DEV_MAG,
    DEV_RTC,
    DEV_IMU,
    DEV_COUNT
} current_device_t;

/*============================================================================*/
/* NMEA PARSING STATE                                                         */
/*============================================================================*/

static const char *nmea_prefixes[NMEA_NUM_PREFIXES] = {"$GNRMC", "$GNGGA", "$GNVTG"};
static const uint8_t prefix_lengths[NMEA_NUM_PREFIXES] = {6, 6, 6};
static uint8_t nmea_buffers[2][NMEA_BUFFER_SIZE];
static volatile uint8_t write_buffer_idx = 0;
static volatile uint16_t nmea_write_idx = 0;
static volatile bool nmea_data_ready = false;
static volatile int8_t ready_buffer_idx = -1;

static volatile struct {
    int8_t matched_prefix;
    uint8_t prefix_pos[NMEA_NUM_PREFIXES];
    bool receiving;
} nmea_state = {-1, {0}, false};

static critical_section_t nmea_crit_sec;

/*============================================================================*/
/* INTER-CORE COMMUNICATION                                                   */
/*============================================================================*/

static TLM_serial_t tx_pool[TX_POOL_SIZE];
static volatile uint8_t tx_index = 0;

void queue_tlm_to_core1(TLM_serial_t *src) {
    uint8_t i = tx_index++ % TX_POOL_SIZE;
    memcpy(&tx_pool[i], src, sizeof(TLM_serial_t));
    multicore_fifo_push_blocking((uint32_t)&tx_pool[i]);
}

void receive_tlm_from_core(TLM_serial_t *dest) {
    TLM_serial_t *src = (TLM_serial_t *)multicore_fifo_pop_blocking();
    memcpy(dest, src, sizeof(TLM_serial_t));
}

/*============================================================================*/
/* HARDWARE INITIALIZATION                                                    */
/*============================================================================*/

static inline void init_i2c(void) {
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void __isr __time_critical_func(uart0_irq_handler)(void) {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);

        if (!nmea_state.receiving) {
            bool prefix_started = false;

            for (int i = 0; i < NMEA_NUM_PREFIXES; i++) {
                uint8_t pos = nmea_state.prefix_pos[i];
                
                if (ch == nmea_prefixes[i][pos]) {
                    nmea_state.prefix_pos[i]++;
                    prefix_started = true;

                    if (nmea_state.prefix_pos[i] == prefix_lengths[i]) {
                        nmea_state.matched_prefix = i;
                        nmea_state.receiving = true;
                        nmea_write_idx = 0;
                        
                        memcpy((void*)nmea_buffers[write_buffer_idx], 
                               nmea_prefixes[i], prefix_lengths[i]);
                        nmea_write_idx = prefix_lengths[i];
                        memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
                        break;
                    }
                } else {
                    nmea_state.prefix_pos[i] = (ch == '$') ? 1 : 0;
                    if (ch == '$') prefix_started = true;
                }
            }

            if (!prefix_started) {
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }

        } else {
            if (nmea_write_idx < NMEA_BUFFER_SIZE - 1) {
                nmea_buffers[write_buffer_idx][nmea_write_idx++] = ch;

                if (ch == '\n') {
                    nmea_buffers[write_buffer_idx][nmea_write_idx] = '\0';
                    ready_buffer_idx = write_buffer_idx;
                    write_buffer_idx ^= 1;
                    nmea_data_ready = true;
                    nmea_state.receiving = false;
                    nmea_state.matched_prefix = -1;
                }
            } else {
                nmea_state.receiving = false;
                nmea_write_idx = 0;
                nmea_state.matched_prefix = -1;
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }
        }
    }
}

static inline void init_uart(void) {
    uart_init(uart0, NEOM8N_UART_BAUDRATE);
    gpio_set_function(NEOM8N_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NEOM8N_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    hw_write_masked(&uart_get_hw(uart0)->ifls,
                    2 << UART_UARTIFLS_RXIFLSEL_LSB,
                    UART_UARTIFLS_RXIFLSEL_BITS);

    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
    irq_set_priority(UART0_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
}

/*============================================================================*/
/* TELEMETRY FUNCTIONS                                                        */
/*============================================================================*/

void telemetry_non_blocking_packet(TLM_packet_t* tlm_packet, current_device_t current_device) {
    switch(current_device) {
        case DEV_GPS:
            if (nmea_data_ready) {
                critical_section_enter_blocking(&nmea_crit_sec);
                if (ready_buffer_idx >= 0) {
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
            qmc5883l_read_mag_drdy(I2C_PORT, &tlm_packet->mag);
            break;

        case DEV_RTC:
            ds3231_read_time(I2C_PORT, &tlm_packet->timeframe);
            break;

        case DEV_IMU:
            // TODO: Implement IMU reading
            break;

        default:
            break;
    }
}

void tlm_serialiser(TLM_packet_t* tlm_packet, TLM_serial_t* tlm_serial) {
    uint16_t crc = 0xFFFF;

    tlm_serial->header = FIFO_HEADER_UINT16;
    memcpy(&tlm_serial->packet, tlm_packet, sizeof(TLM_packet_t));

    uint8_t* data = (uint8_t*)&tlm_serial->packet;
    for (size_t i = 0; i < sizeof(TLM_packet_t); i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) 
                crc = (crc << 1) ^ 0x1021;
            else 
                crc <<= 1;
        }
    }

    tlm_serial->CRC16 = crc;
}

/*============================================================================*/
/* CORE 1 ENTRY POINT                                                         */
/*============================================================================*/

void core1_entry(void) {
    TLM_serial_t received_packet;
    uint32_t packet_count = 0;
    
    while (true) {
        // Receive packet from Core 0
        receive_tlm_from_core(&received_packet);
        packet_count++;
        
        // Verify CRC
        uint16_t calculated_crc = 0xFFFF;
        uint8_t* data = (uint8_t*)&received_packet.packet;
        for (size_t i = 0; i < sizeof(TLM_packet_t); i++) {
            calculated_crc ^= ((uint16_t)data[i] << 8);
            for (uint8_t j = 0; j < 8; j++) {
                if (calculated_crc & 0x8000) 
                    calculated_crc = (calculated_crc << 1) ^ 0x1021;
                else 
                    calculated_crc <<= 1;
            }
        }
        
        bool crc_valid = (calculated_crc == received_packet.CRC16);
        bool header_valid = (received_packet.header == FIFO_HEADER_UINT16);
        
        // Print telemetry on Core 1
			printf("\n========== RCV PACKET ==========\n");
			// RTC timestamp
			printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d %s\n",
			received_packet.packet.timeframe.time.year,
			received_packet.packet.timeframe.time.month,
			received_packet.packet.timeframe.time.day,
			received_packet.packet.timeframe.time.hour,
			received_packet.packet.timeframe.time.minute,
			received_packet.packet.timeframe.time.second,
			received_packet.packet.timeframe.time.is_time_synced ? "[SYNCED]" : "[NOT SYNCED]");
			// Magnetometer readings in raw counts
			printf("[MAG] X:%d Y:%d Z:%d\n",
			received_packet.packet.mag.mag_x,
			received_packet.packet.mag.mag_y,
			received_packet.packet.mag.mag_z);
			// GPS GNRMC - Recomended Minimum Navigation Info
			printf("[GNRMC] Time:%s Date:%s Status:%c Lat:%s%c Lon:%s%c Speed:%.2fkts Course:%.2f°\n",
			received_packet.packet.gnrmc.utc_time,
			received_packet.packet.gnrmc.date,
			received_packet.packet.gnrmc.status,
			received_packet.packet.gnrmc.lat,
			received_packet.packet.gnrmc.ns,
			received_packet.packet.gnrmc.lon,
			received_packet.packet.gnrmc.ew,
			received_packet.packet.gnrmc.speed_knots,
			received_packet.packet.gnrmc.course_deg);
			// GPS GNGGA - Fix quality and altitude data
			printf("[GNGGA] Time:%s Lat:%.6f%c Lon:%.6f%c Fix:%d Sats:%d HDOP:%.2f Alt:%.2fm\n",
			received_packet.packet.gngga.utc_time,
			received_packet.packet.gngga.lat,
			received_packet.packet.gngga.ns,
			received_packet.packet.gngga.lon,
			received_packet.packet.gngga.ew,
			received_packet.packet.gngga.fix_quality,
			received_packet.packet.gngga.num_satellites,
			received_packet.packet.gngga.hdop,
			received_packet.packet.gngga.altitude);
			// GPS GNVTG - Velocity and course data
			printf("[GNVTG] Course(T):%.2f° Course(M):%.2f° Speed:%.2fkts (%.2fkm/h)\n",
			received_packet.packet.gnvtg.course_true,
			received_packet.packet.gnvtg.course_magnetic,
			received_packet.packet.gnvtg.speed_knots,
			received_packet.packet.gnvtg.speed_kmh);
			printf("==========================================\n\n");
					
        // Process packet here (SD card write, etc.)
        // For now, just echo back to Core 0
        queue_tlm_to_core1(&received_packet);
    }
}

/*============================================================================*/
/* MAIN APPLICATION                                                           */
/*============================================================================*/

int main(void) {
    // Initialize USB serial FIRST
    stdio_init_all();
    sleep_ms(2000);  // Give USB time to enumerate
    
    printf("\n========== GNC TEST BENCH v1.2.1 ==========\n");
    
    // Launch Core 1 BEFORE enabling watchdog
    multicore_launch_core1(core1_entry);
    printf("Core 1 launched\n");
    
    // Initialize critical section
    critical_section_init(&nmea_crit_sec);
    
    // Enable watchdog
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    
    // Initialize hardware
    init_i2c();
    init_uart();
    
    // Initialize sensors
    qmc_5883l_init(I2C_PORT);
    ds3231_init(I2C_PORT);
    mpu6050_init(I2C_PORT);
    
    printf("Hardware initialized\n");
    printf("==========================================\n\n");
    
    // Initialize telemetry
    TLM_packet_t tlm_packet = {0};
    TLM_serial_t tlm_serial = {0};
    TLM_serial_t rcv_packet = {0};
    current_device_t current_device = DEV_GPS;
    uint32_t loop_count = 0;
    
    // Main loop
    while (true) {
        watchdog_update();
        
        // Collect sensor data (round-robin)
        telemetry_non_blocking_packet(&tlm_packet, current_device);
        current_device = (current_device + 1) % DEV_COUNT;
        
        // Serialize packet
        tlm_serialiser(&tlm_packet, &tlm_serial);
        
        // Send to Core 1
        queue_tlm_to_core1(&tlm_serial);
        
        // Receive echoed packet from Core 1
        receive_tlm_from_core(&rcv_packet);
        
        loop_count++;
        
        #ifdef DEBUG
        // Print on Core 0 occasionally
        if (loop_count % TELEMETRY_PRINT_INTERVAL == 0) {
            printf("[CORE0] Loop %lu - RTC: %02d:%02d:%02d MAG: X=%d\n",
                loop_count,
                rcv_packet.packet.timeframe.time.hour,
                rcv_packet.packet.timeframe.time.minute,
                rcv_packet.packet.timeframe.time.second,
                rcv_packet.packet.mag.mag_x);
        }
        #endif
    }
    
    return 0;
}