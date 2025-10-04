#include "qmc5883l.h"

// #define DEBUG

int i2c_read_register(i2c_inst_t *i2c, uint8_t addr,uint8_t reg, uint8_t *buf, size_t len) {

    if (i2c_write_blocking(i2c, addr, &reg, 1, true) != 1) {

        return false; 
    }

    if (i2c_read_blocking(i2c, addr, buf, len, false) != (int)len) {

        return false;
    }

    return true;
}

bool i2c_write_register(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) 
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

// read if the chip id is valid and correct

bool qmc5883l_verify_id(i2c_inst_t *i2c) {

    uint8_t buf = 0;
    uint8_t reg = QMC5883L_REG_CHIP_ID;

    if(!i2c_read_register(i2c, QMC5883L_ADDRESS, reg, &buf, 1))
    {
        #ifdef DEBUG
            printf("Failed to verify from QMC5883L\n");
        #endif

        return false;
    }

    return true;
}

//verify if the chip is connected

mag_sensor_state_t qmc_5883l_init(i2c_inst_t *i2c)
{

    
    if(!qmc5883l_verify_id(i2c)) return SENSOR_NOT_DETECTED;

    //set the chip to continuous mode, 200Hz, 2G, 512 oversampling

    uint8_t buf = i2c_read_register(i2c, QMC5883L_ADDRESS, QMC5883L_REG_CONTROL_1, &buf, 1);

    buf = buf | QMC5883_L_MODE_CONTINOUS | QMC5883_L_ODR_200HZ | QMC5883_L_RNG_2G | QMC5883_L_OSR_512;
    
    if(!i2c_write_register(i2c, QMC5883L_ADDRESS, QMC5883L_REG_CONTROL_1, &buf, 1))
    {
        #ifdef DEBUG
            printf("Failed to write to QMC5883L\n");
        #endif

        return SENSOR_ERROR;
    }

    return SENSOR_OK;
}

// Read magnetometer data once the data is ready

mag_sensor_state_t qmc5883l_read_mag_drdy(i2c_inst_t *i2c, qmc_5883_mag_read_t *mag)
{
    uint8_t status_buf = 0;
    uint8_t reg = QMC5883L_REG_STATUS;

    status_buf = i2c_read_register(i2c, QMC5883L_ADDRESS, reg, &status_buf, 1);

    #ifdef DEBUG
        printf("QMC5883L Status: 0x%02X\n", status_buf);
    #endif

    if((status_buf & QMC5883L_STATUS_DRDY) == 0)
    {
        #ifdef DEBUG
            printf("QMC5883L Data not ready\n");
        #endif
        return SENSOR_NDRDY; // data not ready
    }

    if((status_buf & QMC5883L_L_STATUS_OVL) != 0)
    {
        #ifdef DEBUG
            printf("QMC5883L Saturation occurred\n");
        #endif
        return SENSOR_OVERFLOW; // overflow occurred
    }

    uint8_t buf[6];
    reg = QMC5883L_REG_XOUT_L;

    if(!i2c_read_register(i2c, QMC5883L_ADDRESS, reg, buf, 6))
    {
        #ifdef DEBUG
            printf("Failed to read values from QMC5883L\n");
        #endif

        return SENSOR_ERROR;
    }

    mag->mag_x = (int16_t)(buf[1] << 8 | buf[0]);
    mag->mag_y = (int16_t)(buf[3] << 8 | buf[2]);
    mag->mag_z = (int16_t)(buf[5] << 8 | buf[4]);

    return SENSOR_OK;
}