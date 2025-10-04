#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "QMC5883L/qmc5883l.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0

#define I2C_SDA 1
#define I2C_SCL 0

qmc_5883_mag_read_t mag;
int16_t temp;

void init_i2c() {

    // Initialize I2C at 400kHz
    i2c_init(I2C_PORT, 400 * 1000);

    // Set up GPIO pins for I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

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