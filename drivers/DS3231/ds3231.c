#include "ds3231.h"

bool i2c_read_register(i2c_inst_t *i2c, uint8_t addr,uint8_t reg, uint8_t *buf, size_t len) {

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

bool ds3231_verify_id(i2c_inst_t *i2c) {

    uint8_t buf = 0;
    uint8_t reg = DS3231_REG_CHIP_ID;

    if(!i2c_read_register(i2c, DS3231_ADDRESS, reg, &buf, 1))
    {
        #ifdef DEBUG
            printf("Failed to verify from DS3231\n");
        #endif

        return false;
    }

    return buf == DS3231_CHIP_ID;
}

//verify if the chip is connected

rtc_state_t ds3231_init(i2c_inst_t *i2c)
{
    if(!ds3231_verify_id(i2c)) return RTC_NOT_DETECTED;

    //set the control register to enable the oscillator and disable square wave output

    uint8_t buf = 0;
    uint8_t reg = DS3231_REG_CONTROL;

    if(!i2c_read_register(i2c, DS3231_ADDRESS, reg, &buf, 1))
    {
        #ifdef DEBUG
            printf("Failed to read from DS3231\n");
        #endif

        return RTC_ERROR;
    }

    buf = buf & ~DS3231_CONTROL_INTCN; // set INTCN to 0 for square wave output
    buf = buf | DS3231_CONTROL_EOSC;   // enable oscillator

    if(!i2c_write_register(i2c, DS3231_ADDRESS, DS3231_REG_CONTROL, &buf, 1))
    {
        #ifdef DEBUG
            printf("Failed to write to DS3231\n");
        #endif

        return RTC_ERROR;
    }

    return RTC_OK;
}

// Read time from the DS3231 RTC

rtc_state_t ds3231_read_time(i2c_inst_t *i2c, timeframe_rtc_t *timeframe)
{
    uint8_t buf[7];
    uint8_t reg = DS3231_REG_TIME;

    if(!i2c_read_register(i2c, DS3231_ADDRESS, reg, buf, 7))
    {
        #ifdef DEBUG
            printf("Failed to read time from DS3231\n");
        #endif

        return RTC_ERROR;
    }

    // Convert BCD to binary and store in timeframe structure
    timeframe->time.second = ((buf[0] >> 4) * 10) + (buf[0] & 0x0F);
    timeframe->time.minute = ((buf[1] >> 4) * 10) + (buf[1] & 0x0F);
    timeframe->time.hour   = ((buf[2] >> 4) * 10) + (buf[2] & 0x0F);
    timeframe->time.day    = ((buf[4] >> 4) * 10) + (buf[4] & 0x0F);
    timeframe->time.month  = ((buf[5] >> 4) * 10) + (buf[5] & 0x0F);
    timeframe->time.year   = ((buf[6] >> 4) * 10) + (buf[6] & 0x0F) + 2000; // Assuming 21st century

    return RTC_OK;
}

// Write time to the DS3231 RTC

rtc_state_t ds3231_set_time(i2c_inst_t *i2c, const timeframe_rtc_t *timeframe)
{
    uint8_t buf[7];
    uint8_t reg = DS3231_REG_TIME;

    // Convert binary to BCD
    buf[0] = ((timeframe->time.second / 10) << 4) | (timeframe->time.second % 10);
    buf[1] = ((timeframe->time.minute / 10) << 4) | (timeframe->time.minute % 10);
    buf[2] = ((timeframe->time.hour / 10) << 4) | (timeframe->time.hour % 10);
    buf[3] = 0; // Day of week is not used
    buf[4] = ((timeframe->time.day / 10) << 4) | (timeframe->time.day % 10);
    buf[5] = ((timeframe->time.month / 10) << 4) | (timeframe->time.month % 10);
    buf[6] = (((timeframe->time.year - 2000) / 10) << 4) | ((timeframe->time.year - 2000) % 10); // Assuming 21st century

    if(!i2c_write_register(i2c, DS3231_ADDRESS, reg, buf, 7))
    {
        #ifdef DEBUG
            printf("Failed to write time to DS3231\n");
        #endif

        return RTC_ERROR;
    }

    return RTC_OK;
}  

// Read temperature from the DS3231 RTC

rtc_state_t ds3231_read_temperature(i2c_inst_t *i2c, float *temperature)
{
    uint8_t buf[2];
    uint8_t reg = DS3231_REG_TEMP_MSB;

    if(!i2c_read_register(i2c, DS3231_ADDRESS, reg, buf, 2))
    {
        #ifdef DEBUG
            printf("Failed to read temperature from DS3231\n");
        #endif

        return RTC_ERROR;
    }

    // Combine MSB and LSB and convert to Celsius
    int16_t temp_raw = (buf[0] << 8) | buf[1];
    *temperature = temp_raw / 256.0f; // Each increment represents 0.25 degrees Celsius

    return RTC_OK;
}

