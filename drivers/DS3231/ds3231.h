#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

typedef union {
    struct {
        uint16_t second;
        uint16_t minute;
        uint16_t hour;
        uint16_t day;
        uint16_t month;
        uint16_t year;
        bool is_time_synced;  
        uint8_t _padding;     
    } time;                    
    uint16_t raw[7];           
} timeframe_rtc_t;


typedef enum rtc_state {
    RTC_OK = 0,
    RTC_ERROR = -1,
    RTC_NOT_DETECTED = -2,
}rtc_state_t;

#define DS3231_ADDRESS      0x68 // I2C address for DS3231 RTC

#define DS3231_REG_TIME     0x00 // Register address for timekeeping
#define DS3231_REG_CONTROL  0x0E // Register address for control

#define DS3231_REG_STATUS   0x0F // Register address for status

#define DS3231_REG_TEMP_MSB 0x11 // Register address for temperature MSB   
#define DS3231_REG_TEMP_LSB 0x12 // Register address for temperature LSB

#define DS3231_REG_ALARM1   0x07 // Register address for Alarm 1
#define DS3231_REG_ALARM2   0x0B // Register address for Alarm 2

#define DS3231_REG_AGING    0x10 // Register address for aging offset

#define DS3231_REG_CHIP_ID  0xFF // Dummy register for chip ID verification

#define DS3231_CHIP_ID      0x32 // Expected chip ID value for DS3231

#define DS3231_CONTROL_EOSC 0x80 // Enable oscillator

#define DS3231_CONTROL_BBSQW 0x40 // Battery-backed square wave enable

#define DS3231_CONTROL_CONV 0x20 // Convert temperature

#define DS3231_CONTROL_RS2  0x10 // Rate select 2
#define DS3231_CONTROL_RS1  0x08 // Rate select 1

#define DS3231_CONTROL_INTCN 0x04 // Interrupt control

#define DS3231_CONTROL_A2IE 0x02 // Alarm 2 interrupt enable
#define DS3231_CONTROL_A1IE 0x01 // Alarm 1 interrupt enable

#define DS3231_STATUS_OSF   0x80 // Oscillator stop flag
#define DS3231_STATUS_EN32kHz 0x08 // Enable 32kHz output

#define DS3231_STATUS_BSY   0x04 // Busy

#define DS3231_STATUS_A2F   0x02 // Alarm 2 flag
#define DS3231_STATUS_A1F   0x01 // Alarm 1 flag

