#ifndef __IMU_H__
#define __IMU_H__

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/divider.h"
#include <stdbool.h>

#include "main.h"

typedef enum {
    IMU_OK = 0,
    IMU_ERROR = -1,
    IMU_NOT_DETECTED = -2,
} imu_state_t;

typedef struct{
    uint8_t Ax_MSB;
    uint8_t Ax_LSB;

    uint8_t Ay_MSB;
    uint8_t Ay_LSB;
    
    uint8_t Az_MSB;
    uint8_t Az_LSB;
    
    uint8_t Gx_MSB;
    uint8_t Gx_LSB;

    uint8_t Gy_MSB;
    uint8_t Gy_LSB;
    
    uint8_t Gz_MSB;
    uint8_t Gz_LSB;
    
    uint8_t T_MSB;
    uint8_t T_LSB;
} icm42688_raw_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
} icm42688_scaled_t;

#define		IMU_ADDR			0x68

#define 	IMU_CHIP_ID_REG		0x72
#define 	IMU_CHIP_ID			0xE9

#define		IMU_PWR_MGMT		0x10

#define		IMU_ACCEL_CONFIG	0x1B
#define		IMU_GYRO_CONFIG		0x1C
#define 	IMU_FIFO_CONFIG0	0x1D

#define		IMU_SET_G_A_LN		0b00001111

#define		IMU_A_LIMIT_16G		0b00010000

#define		IMU_A_ODR_LN		0b00000101

#define 	IMU_G_UI_FS		    0b00100000

#define 	IMU_G_ODR_LN		0b00000101

#define		IMU_FIFO_DISABLE	0b00000000

#define     IMU_Ax_LSB          0x00

imu_state_t imu_init(i2c_inst_t *i2c);
imu_state_t sensor_value_scaled(i2c_inst_t *i2c, icm42688_scaled_t *imu_scaled);

#endif