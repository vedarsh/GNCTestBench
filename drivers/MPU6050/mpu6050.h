#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

typedef union {
    struct {
        uint16_t accel_x;
        uint16_t accel_y;
        uint16_t accel_z;
        uint16_t gyro_x;
        uint16_t gyro_y;
        uint16_t gyro_z;
    } raw;
    int16_t signed_data[6];
} mpu6050_data_t;

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERROR = -1,
    MPU6050_NOT_DETECTED = -2,
} mpu6050_state_t;

#define MPU6050_ADDRESS         0x68 // I2C address for MPU6050

#define MPU6050_REG_WHO_AM_I   0x75 // Register address for WHO_AM_I
#define MPU6050_WHO_AM_I_ID    0x68 // Expected WHO_AM_I ID for MPU6050
#define MPU6050_REG_PWR_MGMT_1 0x6B // Register address for power management
#define MPU6050_REG_ACCEL_XOUT_H 0x3B // Register address for accelerometer data
#define MPU6050_REG_GYRO_XOUT_H  0x43 // Register address for gyroscope data
#define MPU6050_REG_SMPLRT_DIV   0x19 // Register address for sample rate divider
#define MPU6050_REG_CONFIG       0x1A // Register address for configuration
#define MPU6050_REG_GYRO_CONFIG  0x1B // Register address for gyroscope configuration
#define MPU6050_REG_ACCEL_CONFIG 0x1C // Register address for accelerometer configuration
#define MPU6050_REG_INT_ENABLE   0x38 // Register address for interrupt enable
#define MPU6050_REG_INT_STATUS   0x3A // Register address for interrupt status
#define MPU6050_REG_USER_CTRL    0x6A // Register address for user control
#define MPU6050_REG_FIFO_EN      0x23 // Register address for FIFO enable
#define MPU6050_REG_FIFO_COUNT_H 0x72 // Register address for FIFO count high byte
#define MPU6050_REG_FIFO_R_W     0x74 // Register address for FIFO read
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68 // Register address for signal path reset
#define MPU6050_REG_ACCEL_CONFIG2 0x1D // Register address for accelerometer configuration 2
#define MPU6050_REG_GYRO_CONFIG2  0x1B // Register address for gyroscope configuration 2
#define MPU6050_REG_MOT_THR      0x1F // Register address for motion threshold
#define MPU6050_REG_MOT_DUR      0x20 // Register address for motion
#define MPU6050_REG_FIFO_COUNT_L 0x73 // Register address for FIFO count low byte
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68 // Register address for signal path reset
#define MPU6050_REG_MOT_DETECT_CTRL 0x69 // Register address for motion detection control
#define MPU6050_REG_USER_CTRL    0x6A // Register address for user control
#define MPU6050_REG_PWR_MGMT_2   0x6C // Register address
#define MPU6050_REG_BANK_SEL     0x6D // Register address for bank selection
#define MPU6050_REG_MEM_START_ADDR 0x6E // Register address for memory start address
#define MPU6050_REG_MEM_R_W      0x6F // Register address for memory read/write
#define MPU6050_REG_DMP_CFG_1    0x70 // Register address for DMP configuration 1
#define MPU6050_REG_DMP_CFG_2    0x71 // Register address for DMP configuration 2
#define MPU6050_REG_XA_OFFSET_H  0x77 // Register address for accelerometer X offset high byte
#define MPU6050_REG_XA_OFFSET_L  0x78 // Register address for accelerometer X offset low byte
#define MPU6050_REG_YA_OFFSET_H  0x79 // Register address for accelerometer Y offset high byte
#define MPU6050_REG_YA_OFFSET_L  0x7A // Register address for accelerometer Y offset low byte
#define MPU6050_REG_ZA_OFFSET_H  0x7B // Register address for accelerometer Z offset high byte
#define MPU6050_REG_ZA_OFFSET_L  0x7C // Register address for accelerometer Z offset low byte
#define MPU6050_REG_XG_OFFSET_H  0x13 // Register address for gyroscope X offset high byte
#define MPU6050_REG_XG_OFFSET_L  0x14 // Register address for gyroscope X offset low byte
#define MPU6050_REG_YG_OFFSET_H  0x15 // Register address for gyroscope Y offset high byte
#define MPU6050_REG_YG_OFFSET_L  0x16 // Register address for gyroscope Y offset low byte
#define MPU6050_REG_ZG_OFFSET_H  0x17 // Register address for gyroscope Z offset high byte
#define MPU6050_REG_ZG_OFFSET_L  0x18 // Register address for gyroscope Z offset low byte

#define MPU6050_ACCEL_FS_2G  0x00 // Accelerometer full scale range ±2g
#define MPU6050_ACCEL_FS_4G  0x08 // Accelerometer full scale range ±4g
#define MPU6050_ACCEL_FS_8G  0x10 // Accelerometer full scale range ±8g
#define MPU6050_ACCEL_FS_16G 0x18 // Accelerometer full scale range ±16g

#define MPU6050_GYRO_FS_250DPS  0x00 // Gyroscope full scale range ±250°/s
#define MPU6050_GYRO_FS_500DPS  0x08 // Gyroscope full scale range ±500°/s
#define MPU6050_GYRO_FS_1000DPS 0x10 // Gyroscope full scale range ±1000°/s 
#define MPU6050_GYRO_FS_2000DPS 0x18 // Gyroscope full scale range ±2000°/s 

#define MPU6050_DLPF_260HZ 0x00 // Digital low pass filter setting 260Hz
#define MPU6050_DLPF_184HZ 0x01 // Digital low pass filter setting 184Hz
#define MPU6050_DLPF_94HZ  0x02 // Digital low pass filter setting 94Hz
#define MPU6050_DLPF_44HZ  0x03 // Digital low pass filter setting 44Hz
#define MPU6050_DLPF_21HZ  0x04 // Digital low pass filter setting 21Hz 

#define MPU6050_DLPF_10HZ  0x05 // Digital low pass filter setting 10Hz
#define MPU6050_DLPF_5HZ   0x06 // Digital low pass filter setting 5Hz  

#define MPU6050_TEMP_SENSITIVITY 340.0f // Temperature sensitivity in LSB/°C
#define MPU6050_TEMP_OFFSET      36.53f  // Temperature offset in °C

// Default configuration values for MPU6050
#define MPU6050_DEFAULT_ACCEL_FS      MPU6050_ACCEL_FS_2G
#define MPU6050_DEFAULT_GYRO_FS       MPU6050_GYRO_FS_250DPS
#define MPU6050_DEFAULT_DLPF          MPU6050_DLPF_44HZ
#define MPU6050_DEFAULT_SMPLRT_DIV    0x07 // Sample rate divider
#define MPU6050_PWR_MGMT_1_DEFAULT    0x01 // Power management: use X axis gyroscope as clock
#define MPU6050_CONFIG_DEFAULT        MPU6050_DEFAULT_DLPF
#define MPU6050_GYRO_CONFIG_DEFAULT   MPU6050_DEFAULT_GYRO_FS
#define MPU6050_ACCEL_CONFIG_DEFAULT  MPU6050_DEFAULT_ACCEL_FS

mpu6050_state_t mpu6050_init(i2c_inst_t *i2c);
mpu6050_state_t mpu6050_read_data(i2c_inst_t *i2c, mpu6050_data_t *data);
mpu6050_state_t mpu6050_read_temp(i2c_inst_t *i2c, float *temperature);