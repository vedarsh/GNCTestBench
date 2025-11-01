#include "ICM45686.h"


#define SCALE 1000
#define ACCEL_SENS 16384   // Example for ±2g
#define GYRO_SENS 131      // Example for ±1000 dps
#define TEMP_DIVISOR 132


static inline bool i2c_read_register(i2c_inst_t *i2c, uint8_t addr,uint8_t reg, uint8_t *buf, size_t len) {

    if (i2c_write_blocking(i2c, addr, &reg, 1, true) != 1) 
    {

        return false; 
    }

    if (i2c_read_blocking(i2c, addr, buf, len, false) != (int)len) 
    {

        return false;
    }

    return true;
}

static inline bool i2c_write_register(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) 
{

    uint8_t buf[len + 1];

    buf[0] = reg;

    for (size_t i = 0; i < len; i++) 
    {
        buf[i + 1] = data[i];
    }

    if (i2c_write_blocking(i2c, addr, buf, len + 1, false) != (int)(len + 1)) 
    {
        return false;
    }

    return true;
}

imu_state_t imu_init(i2c_inst_t *i2c)
{
    //Check if the chip ID is correct
    uint8_t who_am_i = 0;
    
    if (!i2c_read_register(i2c, IMU_ADDR, IMU_CHIP_ID_REG, &who_am_i, 1))
    {
        return IMU_NOT_DETECTED;
    }

    if(who_am_i != IMU_CHIP_ID)
    {
        return IMU_ERROR;
    }

    uint8_t buf = 0x00;

    //Write the acceleration max value to 16G and acceleration sample rate to 1.6KHz
    if(!i2c_read_register(i2c, IMU_ADDR, IMU_ACCEL_CONFIG, &buf, sizeof(buf)))
    {
        return IMU_ERROR;
    }
    
    buf = buf | IMU_A_LIMIT_16G | IMU_A_ODR_LN;

    i2c_write_register(i2c, IMU_ADDR, IMU_ACCEL_CONFIG, &buf, sizeof(buf));

    buf = 0;


    //Write the Gyro max rotation rate to 1000 DPS and Gyro sample rate to 1.6KHz
    if(!i2c_read_register(i2c, IMU_ADDR, IMU_GYRO_CONFIG, &buf, sizeof(buf)))
    {
        return IMU_ERROR;
    }
    
    buf = buf | IMU_G_UI_FS | IMU_G_ODR_LN;

    i2c_write_register(i2c, IMU_ADDR, IMU_GYRO_CONFIG, &buf, sizeof(buf));

    buf = 0;

    //Set the Accelerometer and Gyroscope to Low Noise Mode

    if(!i2c_read_register(i2c, IMU_ADDR, IMU_PWR_MGMT, &buf, sizeof(buf)))
    {
        return IMU_ERROR;
    }
    
    buf = buf | IMU_SET_G_A_LN;

    i2c_write_register(i2c, IMU_ADDR, IMU_GYRO_CONFIG, &buf, sizeof(buf));

    buf = 0;

    return IMU_OK;

}

// function to sample the data
imu_state_t read_sensor_value(i2c_inst_t *i2c, icm42688_raw_t *imu_raw)
{
    uint8_t buf[14];

    if(!i2c_read_register(i2c, IMU_ADDR, IMU_Ax_LSB, buf, sizeof(buf)))
    {
        return IMU_ERROR;
    }

    buf[0] = imu_raw -> Ax_MSB;
    buf[1] = imu_raw -> Ax_LSB;
    buf[2] = imu_raw -> Ay_MSB;
    buf[3] = imu_raw -> Ay_LSB;
    buf[4] = imu_raw -> Az_MSB;
    buf[5] = imu_raw -> Az_LSB;

    
    buf[6] = imu_raw -> Gx_MSB;
    buf[7] = imu_raw -> Gx_LSB;
    buf[8] = imu_raw -> Gy_MSB;
    buf[9] = imu_raw -> Gy_LSB;
    buf[10] = imu_raw -> Gz_MSB;
    buf[11] = imu_raw -> Gz_LSB;

    buf[12] = imu_raw -> T_MSB;
    buf[13] = imu_raw -> T_LSB;

    return IMU_OK;
}


imu_state_t sensor_value_scaled(i2c_inst_t *i2c, icm42688_scaled_t *imu_scaled)
{
    icm42688_raw_t imu_read;

    // Read raw sensor values first
    if(!i2c_read_register(i2c, IMU_ADDR, IMU_Ax_LSB, (uint8_t *) &imu_read, sizeof(imu_read))) 
    {
        return IMU_ERROR; 
    }

    // Combine MSB and LSB into signed 16-bit integers
    int16_t Ax_raw = (int16_t)((imu_read.Ax_MSB << 8) | imu_read.Ax_LSB);
    int16_t Ay_raw = (int16_t)((imu_read.Ay_MSB << 8) | imu_read.Ay_LSB);
    int16_t Az_raw = (int16_t)((imu_read.Az_MSB << 8) | imu_read.Az_LSB);

    int16_t Gx_raw = (int16_t)((imu_read.Gx_MSB << 8) | imu_read.Gx_LSB);
    int16_t Gy_raw = (int16_t)((imu_read.Gy_MSB << 8) | imu_read.Gy_LSB);
    int16_t Gz_raw = (int16_t)((imu_read.Gz_MSB << 8) | imu_read.Gz_LSB);

    int16_t T_raw  = (int16_t)((imu_read.T_MSB << 8) | imu_read.T_LSB);

    // --- Accelerometer (in g) ---
    imu_scaled->accel_x_g = (float)Ax_raw / ACCEL_SENS;
    imu_scaled->accel_y_g = (float)Ay_raw / ACCEL_SENS;
    imu_scaled->accel_z_g = (float)Az_raw / ACCEL_SENS;

    // --- Gyroscope (in °/s) ---
    imu_scaled->gyro_x_dps = (float)Gx_raw / GYRO_SENS;
    imu_scaled->gyro_y_dps = (float)Gy_raw / GYRO_SENS;
    imu_scaled->gyro_z_dps = (float)Gz_raw / GYRO_SENS;

    // --- Temperature (in °C) ---
    imu_scaled->temp_c = ((float)T_raw / TEMP_DIVISOR) + 25.0f;

    return IMU_OK;
}
