#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "QMC5883L/qmc5883l.h"
#include "DS3231/ds3231.h"
#include "NEOM8N/neom8n.h"

#define I2C_PORT i2c0

#define I2C_SDA 1
#define I2C_SCL 0

qmc_5883_mag_read_t mag;
int16_t temp;

#define NEOM8N_UART_TX_PIN 12
#define NEOM8N_UART_RX_PIN 13

void init_i2c() {

    // Initialize I2C at 400kHz
    i2c_init(I2C_PORT, 400 * 1000);

    // Set up GPIO pins for I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

}

void init_uart() {

    // Initialize UART
    uart_init(uart0, 9600);

    // Set up GPIO pins for UART
    gpio_set_function(NEOM8N_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NEOM8N_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);  
}


int main()
{
    stdio_init_all();

    init_i2c();

    qmc_5883l_init(I2C_PORT);

    while (true) {
        
        sleep_ms(100);
    }
}