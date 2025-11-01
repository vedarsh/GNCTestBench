#include "ICM45686.h"

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

imu_state_t imu_init(i2c_inst_t *i2c)
 {
    uint8_t who_am_i = 0;
    
    if (!i2c_read_register(i2c, IMU_ADDR, IMU_CHIP_ID_REG, &who_am_i, 1))
    {
        return IMU_NOT_DETECTED;
    }

    if(who_am_i != IMU_CHIP_ID)
    {
        return IMU_ERROR;
    }

 }

imu_state_t imu_accel_config(i2c_inst_t *i2c)
{
    uint8_t buf = 0x00;

    if(!i2c_read_register(i2c, IMU_ADDR, IMU_ACCEL_CONFIG, &buf, sizeof(buf)))
    {
        return IMU_ERROR;
    }

    
}