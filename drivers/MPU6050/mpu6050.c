#include "mpu6050.h"

static inline bool i2c_read_register(i2c_inst_t *i2c, uint8_t addr,uint8_t reg, uint8_t *buf, size_t len) {

    if (i2c_write_blocking(i2c, addr, &reg, 1, true) != 1) {

        return false; 
    }

    if (i2c_read_blocking(i2c, addr, buf, len, false) != (int)len) {

        return false;
    }

    return true;
}

static inline bool i2c_write_register(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) 
{

    uint8_t buf[len + 1];
    buf[0] = reg;

    for (size_t i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }

    if (i2c_write_blocking(i2c, addr, buf, len + 1, false) != (int)(len + 1)) {
        return false;
    }

    return true;
}

mpu6050_state_t mpu6050_init(i2c_inst_t *i2c)
{
    // Check device ID
    uint8_t who_am_i = 0;
    if (!i2c_read_register(i2c, MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I, &who_am_i, 1)) 
    {
        return MPU6050_ERROR;
    }
    if (who_am_i != MPU6050_WHO_AM_I_ID) 
    {
        return MPU6050_NOT_DETECTED;
    }

    // Write default values as per mpu6050.h
    uint8_t pwr_mgmt_1 = MPU6050_PWR_MGMT_1_DEFAULT;
    if (!i2c_write_register(i2c, MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, &pwr_mgmt_1, 1)) 
    {
        return MPU6050_ERROR;
    }

    uint8_t smplrt_div = MPU6050_DEFAULT_SMPLRT_DIV;
    if (!i2c_write_register(i2c, MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV, &smplrt_div, 1)) 
    {
        return MPU6050_ERROR;
    }

    uint8_t config = MPU6050_CONFIG_DEFAULT;
    if (!i2c_write_register(i2c, MPU6050_ADDRESS, MPU6050_REG_CONFIG, &config, 1)) 
    {
        return MPU6050_ERROR;
    }

    uint8_t gyro_config = MPU6050_GYRO_CONFIG_DEFAULT;
    if (!i2c_write_register(i2c, MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, &gyro_config, 1)) 
    {
        return MPU6050_ERROR;
    }

    uint8_t accel_config = MPU6050_ACCEL_CONFIG_DEFAULT;
    if (!i2c_write_register(i2c, MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, &accel_config, 1)) 
    {
        return MPU6050_ERROR;
    }

    return MPU6050_OK;
}

mpu6050_state_t mpu6050_read_data(i2c_inst_t *i2c, mpu6050_data_t *data) 
{
    uint8_t buf[14];
    if (!i2c_read_register(i2c, MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, buf, 14)) 
    {
        return MPU6050_ERROR;
    }

    data->raw.accel_x = (buf[0] << 8) | buf[1];
    data->raw.accel_y = (buf[2] << 8) | buf[3];
    data->raw.accel_z = (buf[4] << 8) | buf[5];
    data->raw.gyro_x  = (buf[8] << 8) | buf[9];
    data->raw.gyro_y  = (buf[10] << 8) | buf[11];
    data->raw.gyro_z  = (buf[12] << 8) | buf[13];

    return MPU6050_OK;
}

mpu6050_state_t mpu6050_read_temp(i2c_inst_t *i2c, float *temperature) 
{
    uint8_t buf[2];
    if (!i2c_read_register(i2c, MPU6050_ADDRESS, 0x41, buf, 2)) 
    {
        return MPU6050_ERROR;
    }

    int16_t raw_temp = (buf[0] << 8) | buf[1];
    *temperature = (raw_temp / 340.0f) + 36.53f;

    return MPU6050_OK;
}