#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "QMC5883L/qmc5883l.h"
#include "DS3231/ds3231.h"
#include "hardware/dma.h"
#include "MPU6050/mpu6050.h"
#include "NEOM8N/neom8n.h"

#define I2C_PORT i2c0

#define I2C_SDA 1
#define I2C_SCL 0

#define NEOM8N_UART_TX_PIN 12
#define NEOM8N_UART_RX_PIN 13

#define NEOM8N_UART_BAUDRATE 115200

#define NMEA_BUFFER_SIZE 128
#define NMEA_NUM_PREFIXES 3

const char *nmea_prefixes[NMEA_NUM_PREFIXES] = {
    "$GNRMC", "$GNGGA", "$GNVTG"
};
volatile uint8_t nmea_rx_buffer[NMEA_BUFFER_SIZE];
volatile uint16_t nmea_index = 0;
volatile bool nmea_data_ready = false;

typedef struct TLM_packet
{
    qmc_5883_mag_read_t mag;
    timeframe_rtc_t timeframe;
    nmea_gnrmc_t gnrmc;
    nmea_gngga_t gngga;
    nmea_gnvtg_t gnvtg;
} TLM_packet_t;

typedef enum {
    DEV_GPS = 0,
    DEV_MAG,
    DEV_RTC,
    DEV_IMU,
    DEV_NONE,
    DEV_COUNT
} current_device_t;

void init_i2c() {

    // Initialize I2C at 400kHz
    i2c_init(I2C_PORT, 400 * 1000);

    // Set up GPIO pins for I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

}

static int matched_prefix = -1;
static bool receiving = false;

#define MAX_PREFIX_LEN 6

// per-prefix match lengths
static uint8_t prefix_match_len[NMEA_NUM_PREFIXES] = {0};

void uart0_irq_routine(void) {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);

        if (!receiving) {
            bool any_match = false;

            for (int i = 0; i < NMEA_NUM_PREFIXES; i++) {
                const char *prefix = nmea_prefixes[i];

                if (ch == prefix[prefix_match_len[i]]) {
                    // character matches current prefix
                    prefix_match_len[i]++;
                    any_match = true;

                    if (prefix_match_len[i] == strlen(prefix)) {
                        // Full prefix matched
                        matched_prefix = i;
                        receiving = true;
                        nmea_index = 0;
                        memcpy((void*)nmea_rx_buffer, prefix, prefix_match_len[i]);
                        nmea_index = prefix_match_len[i];

                        // reset all prefix trackers
                        for (int j = 0; j < NMEA_NUM_PREFIXES; j++) prefix_match_len[j] = 0;
                        break;
                    }
                } else {
                    // mismatch: reset only this prefix tracker
                    prefix_match_len[i] = (ch == '$') ? 1 : 0;
                }
            }

            if (!any_match) {
                // no prefix matched: reset all
                for (int i = 0; i < NMEA_NUM_PREFIXES; i++) prefix_match_len[i] = 0;
            }

        } else {
            // Receiving sentence
            if (nmea_index < NMEA_BUFFER_SIZE - 1) {
                nmea_rx_buffer[nmea_index++] = ch;

                if (ch == '\n') {
                    nmea_rx_buffer[nmea_index] = '\0';
                    nmea_data_ready = true;
                    receiving = false;
                    matched_prefix = -1;
                }
            } else {
                // overflow
                receiving = false;
                nmea_index = 0;
                matched_prefix = -1;
                for (int i = 0; i < NMEA_NUM_PREFIXES; i++) prefix_match_len[i] = 0;
            }
        }
    }
}

void init_uart() 
{

    // Initialize UART
    uart_init(uart0, 9600);

    // Set up GPIO pins for UART
    gpio_set_function(NEOM8N_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NEOM8N_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_routine);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
}

current_device_t current_device = DEV_GPS;

void telemetry_non_blocking_packet(TLM_packet_t* tlm_packet)
{

    switch(current_device) {
        case DEV_GPS:
            if (nmea_data_ready) {
                nmea_data_ready = false;

                // Copy to local buffer to prevent ISR overwrite
                char nmea_sentence[NMEA_BUFFER_SIZE];
                memcpy(nmea_sentence, (const void*)nmea_rx_buffer, nmea_index);
                printf("Received NMEA: %s\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", nmea_sentence);
                nmea_type_t type = nmea_parse_sentence(nmea_sentence, &tlm_packet->gnrmc, &tlm_packet->gngga, &tlm_packet->gnvtg);

                nmea_index = 0; // Reset buffer index for next sentence
            }
            break;

        case DEV_MAG:
            qmc5883l_read_mag_drdy(I2C_PORT, &tlm_packet->mag);
            break; 

        case DEV_RTC:
            ds3231_read_time(I2C_PORT, &tlm_packet->timeframe);
            break;  
        case DEV_IMU:
            //TBD after IMU is fixed
            break;
        case DEV_NONE:
            break;
    }

    current_device = (current_device + 1) % DEV_NONE;

}

int main()
{
    stdio_init_all();

    init_i2c();

    init_uart();

    //uart_rx_dma_init(uart0, nmea_rx_buffer, NMEA_BUFFER_SIZE, NULL);

    qmc_5883l_init(I2C_PORT);
    ds3231_init(I2C_PORT);
    mpu6050_init(I2C_PORT);

    mpu6050_data_t mpu_data;
    TLM_packet_t tlm_packet;

    while (true) 
    {
        telemetry_non_blocking_packet(&tlm_packet);

        printf("Mag: X=%d Y=%d Z=%d | ", tlm_packet.mag.mag_x, tlm_packet.mag.mag_y, tlm_packet.mag.mag_z);
        printf("RTC: %02d:%02d:%02d %02d/%02d/%04d | ", tlm_packet.timeframe.time.hour, tlm_packet.timeframe.time.minute, tlm_packet.timeframe.time.second,
               tlm_packet.timeframe.time.day, tlm_packet.timeframe.time.month, tlm_packet.timeframe.time.year);
        printf("GNRMC: Time=%s Status=%c Lat=%s %c Lon=%s %c Speed=%.2f Course=%.2f Date=%s | ",
               tlm_packet.gnrmc.utc_time, tlm_packet.gnrmc.status, tlm_packet.gnrmc.lat, tlm_packet.gnrmc.ns,
               tlm_packet.gnrmc.lon, tlm_packet.gnrmc.ew, tlm_packet.gnrmc.speed_knots,
               tlm_packet.gnrmc.course_deg, tlm_packet.gnrmc.date);
        printf("GNGGA: Time=%s Lat=%.6f %c Lon=%.6f %c Fix=%d Sats=%d HDOP=%.1f Alt=%.1f | ",
               tlm_packet.gngga.utc_time, tlm_packet.gngga.lat, tlm_packet.gngga.ns,
               tlm_packet.gngga.lon, tlm_packet.gngga.ew, tlm_packet.gngga.fix_quality,
               tlm_packet.gngga.num_satellites, tlm_packet.gngga.hdop,
               tlm_packet.gngga.altitude);
        printf("GNVTG: CourseT=%.2f CourseM=%.2f SpeedKnots=%.2f SpeedKmh=%.2f\n",
               tlm_packet.gnvtg.course_true, tlm_packet.gnvtg.course_magnetic,
               tlm_packet.gnvtg.speed_knots, tlm_packet.gnvtg.speed_kmh);
    }

}


