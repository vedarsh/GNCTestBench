#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

typedef union {
    struct {
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;
    };
    int16_t raw[3];  // same 6 bytes accessible as array
} qmc_5883_mag_read_t;

typedef enum mag_sensor_state {
    SENSOR_OK = 0,
    SENSOR_ERROR = -1,
    SENSOR_NOT_DETECTED = -2,
    SENSOR_OVERFLOW = -3,
    SENSOR_NDRDY= -4
}mag_sensor_state_t;

#define     QMC5883L_MODE_CHIP_ID   0xFF;

#define     QMC5883L_ADDRESS        0x0D

#define     QMC5883L_REG_XOUT_L     0x00
#define     QMC5883L_REG_XOUT_H     0x01

#define     QMC5883L_REG_YOUT_L     0x02
#define     QMC5883L_REG_YOUT_H     0x03

#define     QMC5883L_REG_ZOUT_L     0x04
#define     QMC5883L_REG_ZOUT_H     0x05

#define     QMC5883L_REG_STATUS     0x06

#define     QMC5883L_REG_TEMP_L     0x07
#define     QMC5883L_REG_TEMP_H     0x08

#define     QMC5883L_REG_CONTROL_1  0x09
#define     QMC5883L_REG_CONTROL_2  0x0A

#define     QMC5883L_REG_SET_RESET  0x0B 

#define     QMC5883L_REG_CHIP_ID    0x0D

#define    QMC5883L_STATUS_DRDY      0x01
#define    QMC5883L_L_STATUS_OVL     0x02
#define    QMC5883L_L_STATUS_DOR     0x04

#define     QMC5883_L_MODE_STANDBY   0x00
#define     QMC5883_L_MODE_CONTINOUS 0x01

#define     QMC5883_L_ODR_10HZ    0x00
#define     QMC5883_L_ODR_50HZ    0x04
#define     QMC5883_L_ODR_100HZ   0x08
#define     QMC5883_L_ODR_200HZ   0x0C

#define     QMC5883_L_RNG_2G      0x00
#define     QMC5883_L_RNG_8G      0x10

#define     QMC5883_L_OSR_512     0x00
#define     QMC5883_L_OSR_256     0x40
#define     QMC5883_L_OSR_128     0x80
#define     QMC5883_L_OSR_64      0xC0

bool qmc5883l_verify_id(i2c_inst_t *i2c);
mag_sensor_state_t qmc_5883l_init(i2c_inst_t *i2c);
mag_sensor_state_t qmc5883l_read_mag_drdy(i2c_inst_t *i2c, qmc_5883_mag_read_t *mag);
mag_sensor_state_t qmc5883l_read_temp(i2c_inst_t *i2c, int16_t *temp);