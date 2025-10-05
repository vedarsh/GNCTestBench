#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "QMC5883L/qmc5883l.h"
#include "DS3231/ds3231.h"
#include "hardware/dma.h"
#include "MPU6050/mpu6050.h"

#define I2C_PORT i2c0

#define I2C_SDA 1
#define I2C_SCL 0

#define NEOM8N_UART_TX_PIN 12
#define NEOM8N_UART_RX_PIN 13

#define NEOM8N_UART_BAUDRATE 9600

#define NMEA_BUFFER_SIZE  82 // Standard NMEA sentence max length
#define NMEA_SENTENCE_START '$'
#define NMEA_SENTENCE_END   '\n'

qmc_5883_mag_read_t mag;
int16_t temp;

timeframe_rtc_t timeframe;

uint8_t nmea_rx_buffer[NMEA_BUFFER_SIZE];
volatile bool nmea_data_ready = false;


void init_i2c() {

    // Initialize I2C at 400kHz
    i2c_init(I2C_PORT, 400 * 1000);

    // Set up GPIO pins for I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

}

#define NMEA_BUFFER_SIZE 82
uint8_t nmea_buf1[NMEA_BUFFER_SIZE];
uint8_t nmea_buf2[NMEA_BUFFER_SIZE];

volatile uint8_t* current_buf = nmea_buf1;
volatile uint8_t* processing_buf = nmea_buf2;
volatile uint16_t nmea_index = 0;

static bool receiving = false;

void uart0_irq_routine(void) {
    static bool receiving = false;

    while(uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);

        if (ch == '$') {             // start of sentence
            receiving = true;
            nmea_index = 0;
            nmea_rx_buffer[nmea_index++] = ch;
        }
        else if (receiving) {
            if (nmea_index < NMEA_BUFFER_SIZE - 1) {
                nmea_rx_buffer[nmea_index++] = ch;

                if (ch == '\n') {   // end of sentence
                    nmea_data_ready = true;
                    receiving = false;
                }
            } else {
                // buffer overflow, discard
                receiving = false;
                nmea_index = 0;
            }
        }
        // ignore bytes until next $
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

    while (true) {
        //read and print data from MPU6050
        int mpu_status = mpu6050_read_data(I2C_PORT, &mpu_data);
        if (mpu_status == MPU6050_OK) {
            printf("MPU6050 Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n",
                   mpu_data.raw.accel_x, mpu_data.raw.accel_y, mpu_data.raw.accel_z,
                   mpu_data.raw.gyro_x, mpu_data.raw.gyro_y, mpu_data.raw.gyro_z);
        } else {
            printf("Failed to read MPU6050 data, status: %d\n", mpu_status);
        }
        sleep_ms(100);
    }
}