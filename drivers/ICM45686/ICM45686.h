#ifndef __IMU_H__
#define __IMU_H__

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

typedef enum {
    IMU_OK = 0,
    IMU_ERROR = -1,
    IMU_NOT_DETECTED = -2,
} imu_state_t;

#define		IMU_ADDR			0x68

#define 	IMU_CHIP_ID_REG		0x72
#define 	IMU_CHIP_ID			0xE9

#define		IMU_ACCEL_CONFIG	0x1B
#define		IMU_GYRO_CONFIG		0x1C
#define 	IMU_FIFO_CONFIG0	0x1D

#define		IMU_G_LIMIT_16G		0b00010000

#define		IMU_G_ODR_LN		0b00000101

#define 	IMU_GYRO_UI_FS		0b00100000

#define 	IMU_GYRO_ODR_LN		0b00000101

#define		IMU_FIFO_DISABLE	0b00000000



#endif