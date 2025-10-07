/**
 * @file main.c
 * @brief GNC Test Bench Telemetry System - Aerospace Compliant Implementation
 * @author Vedarsh
 * @date 2025-10-07
 * @version 2.0.0
 * @compliance DO-178C Level C, MISRA-C:2012
 * 
 * @note Dual-core architecture:
 *       Core 0: Sensor acquisition, fusion, and filtering
 *       Core 1: Telemetry processing and output
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "hardware/divider.h"

#include "QMC5883L/qmc5883l.h"
#include "DS3231/ds3231.h"
#include "MPU6050/mpu6050.h"
#include "NEOM8N/neom8n.h"

/*============================================================================*/
/* CONFIGURATION PARAMETERS                                                   */
/*============================================================================*/

#define I2C_PORT i2c0
#define I2C_SDA 1
#define I2C_SCL 0
#define I2C_TIMEOUT_US 50000U
#define NEOM8N_UART_TX_PIN 12
#define NEOM8N_UART_RX_PIN 13
#define NEOM8N_UART_BAUDRATE 9600U
#define NMEA_BUFFER_SIZE 128U
#define NMEA_NUM_PREFIXES 3U
#define MAX_PREFIX_LEN 6U
#define GPS_SYNC_TIMEOUT_MS 120000U
#define TELEMETRY_CYCLE_MS 100U
#define WATCHDOG_TIMEOUT_MS 8000U
#define FIFO_HEADER_UINT16 0xABABU
#define FIFO_FOOTER_UINT16 0xCDCDU
#define TX_POOL_SIZE 8U
#define RX_POOL_SIZE 8U

/* Sensor fusion parameters */
#define ACCEL_WEIGHT 0.02f
#define GYRO_DT 0.01f  /* 100Hz update rate */
#define MAG_DECLINATION 0.0f  /* Adjust for local magnetic declination */
#define COMPLEMENTARY_ALPHA 0.98f

/* Fault detection thresholds */
#define MAX_ACCEL_G 16.0f
#define MAX_GYRO_DPS 2000.0f
#define MAX_MAG_GAUSS 8.0f
#define SENSOR_TIMEOUT_MS 500U
#define MAX_CRC_ERRORS 10U
#define MAX_CONSECUTIVE_FAULTS 5U

/* ICM42688 dummy conversion factors (will be replaced with actual driver) */
#define ICM42688_ACCEL_SCALE 0.00059855f  /* 16g range: 32768/16 */
#define ICM42688_GYRO_SCALE 0.061037f     /* 2000dps range: 32768/2000 */

/*============================================================================*/
/* CCSDS SPACE PACKET PROTOCOL DEFINITIONS                                    */
/*============================================================================*/

/* CCSDS Packet Version Numbers */
#define CCSDS_VERSION_1 0x00

/* CCSDS Packet Types */
#define CCSDS_TYPE_TM 0x00  /* Telemetry */
#define CCSDS_TYPE_TC 0x01  /* Telecommand */

/* CCSDS Secondary Header Flags */
#define CCSDS_SEC_HDR_NOT_PRESENT 0x00
#define CCSDS_SEC_HDR_PRESENT 0x01

/* CCSDS Application Process Identifiers (APID) */
#define CCSDS_APID_GNC_TLM 0x100  /* GNC Telemetry */
#define CCSDS_APID_GNC_CMD 0x101  /* GNC Commands */

/* CCSDS Packet Data Field Header */
#pragma pack(push,1)
typedef struct {
    uint16_t packet_id;          /* Version(3) + Type(1) + SecHdr(1) + APID(11) */
    uint16_t packet_seq_ctrl;    /* Seq Flags(2) + Seq Count(14) */
    uint16_t packet_data_length; /* Length of data field minus 1 */
} ccsds_primary_header_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    uint32_t coarse_time;  /* Seconds since epoch */
    uint16_t fine_time;    /* Sub-seconds (65536ths) */
} ccsds_time_t;
#pragma pack(pop)

/*============================================================================*/
/* SAFETY AND HEALTH MONITORING                                               */
/*============================================================================*/

typedef enum {
    HEALTH_NOMINAL = 0,
    HEALTH_DEGRADED = 1,
    HEALTH_CRITICAL = 2,
    HEALTH_FAILED = 3
} health_status_t;

typedef struct {
    health_status_t gps;
    health_status_t mag;
    health_status_t rtc;
    health_status_t imu;
    health_status_t fusion;
    uint32_t crc_error_count;
    uint32_t sensor_timeout_count;
    uint32_t fusion_divergence_count;
    absolute_time_t last_gps_update;
    absolute_time_t last_mag_update;
    absolute_time_t last_imu_update;
} system_health_t;

/*============================================================================*/
/* DATA STRUCTURES - PACKED FOR DETERMINISTIC SIZING                          */
/*============================================================================*/

// #pragma pack(push,1)
// typedef struct {
//     int16_t accel_x_raw;
//     int16_t accel_y_raw;
//     int16_t accel_z_raw;
//     int16_t gyro_x_raw;
//     int16_t gyro_y_raw;
//     int16_t gyro_z_raw;
//     int16_t temp_raw;
// } icm42688_raw_t;
// #pragma pack(pop)

// #pragma pack(push,1)
// typedef struct {
//     float accel_x_g;
//     float accel_y_g;
//     float accel_z_g;
//     float gyro_x_dps;
//     float gyro_y_dps;
//     float gyro_z_dps;
//     float temp_c;
// } icm42688_scaled_t;
// #pragma pack(pop)

// #pragma pack(push,1)
// typedef struct {
//     float roll_deg;
//     float pitch_deg;
//     float yaw_deg;
//     float roll_rate_dps;
//     float pitch_rate_dps;
//     float yaw_rate_dps;
//     float accel_magnitude_g;
//     float heading_mag_deg;
//     bool fusion_valid;
// } attitude_data_t;
// #pragma pack(pop)

// #pragma pack(push,1)
// typedef struct {
//     qmc_5883_mag_read_t mag;
//     timeframe_rtc_t timeframe;
//     nmea_gnrmc_t gnrmc;
//     nmea_gngga_t gngga;
//     nmea_gnvtg_t gnvtg;
//     icm42688_raw_t imu_raw;
//     icm42688_scaled_t imu_scaled;
//     attitude_data_t attitude;
//     system_health_t health;
//     uint32_t packet_sequence;
//     uint32_t core0_timestamp_ms;
// } TLM_packet_t;
// #pragma pack(pop)

// #pragma pack(push,1)
// typedef struct {
//     uint16_t header;
//     TLM_packet_t packet;
//     uint16_t crc16;
//     uint16_t footer;
// } TLM_serial_t;
// #pragma pack(pop)

// typedef enum {
//     DEV_GPS = 0,
//     DEV_MAG = 1,
//     DEV_RTC = 2,
//     DEV_IMU = 3,
//     DEV_FUSION = 4,
//     DEV_COUNT = 5
// } current_device_t;

/*============================================================================*/
/* DATA STRUCTURES - CCSDS COMPLIANT                                          */
/*============================================================================*/

#pragma pack(push,1)
typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temp_raw;
} icm42688_raw_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
} icm42688_scaled_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float accel_magnitude_g;
    float heading_mag_deg;
    bool fusion_valid;
} attitude_data_t;
#pragma pack(pop)

/* Simplified OBC telemetry packet */
#pragma pack(push,1)
typedef struct {
    ccsds_time_t timestamp;
    
    /* Attitude and rates */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    
    /* GPS essentials */
    float latitude_deg;
    float longitude_deg;
    float altitude_m;
    float speed_mps;
    uint8_t gps_fix_quality;
    uint8_t gps_num_sats;
    
    /* Sensor raw data */
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    
    /* System health */
    uint8_t health_status;  /* Packed health flags */
    uint32_t packet_count;
} obc_telemetry_t;
#pragma pack(pop)

/* CCSDS compliant telemetry packet */
#pragma pack(push,1)
typedef struct {
    ccsds_primary_header_t header;
    obc_telemetry_t telemetry;
    uint16_t crc16;
} ccsds_telemetry_packet_t;
#pragma pack(pop)

/* Full internal telemetry packet */
#pragma pack(push,1)
typedef struct {
    qmc_5883_mag_read_t mag;
    timeframe_rtc_t timeframe;
    nmea_gnrmc_t gnrmc;
    nmea_gngga_t gngga;
    nmea_gnvtg_t gnvtg;
    icm42688_raw_t imu_raw;
    icm42688_scaled_t imu_scaled;
    attitude_data_t attitude;
    system_health_t health;
    uint32_t packet_sequence;
    uint32_t core0_timestamp_ms;
} TLM_packet_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    uint16_t header;
    TLM_packet_t packet;
    uint16_t crc16;
    uint16_t footer;
} TLM_serial_t;
#pragma pack(pop)

typedef enum {
    DEV_GPS = 0,
    DEV_MAG = 1,
    DEV_RTC = 2,
    DEV_IMU = 3,
    DEV_FUSION = 4,
    DEV_COUNT = 5
} current_device_t;


/*============================================================================*/
/* NMEA PARSING STATE                                                         */
/*============================================================================*/

static const char *nmea_prefixes[NMEA_NUM_PREFIXES] = {"$GNRMC", "$GNGGA", "$GNVTG"};
static const uint8_t prefix_lengths[NMEA_NUM_PREFIXES] = {6U, 6U, 6U};
static uint8_t nmea_buffers[2][NMEA_BUFFER_SIZE];
static volatile uint8_t write_buffer_idx = 0U;
static volatile uint16_t nmea_write_idx = 0U;
static volatile bool nmea_data_ready = false;
static volatile int8_t ready_buffer_idx = -1;

static volatile struct {
    int8_t matched_prefix;
    uint8_t prefix_pos[NMEA_NUM_PREFIXES];
    bool receiving;
} nmea_state = {-1, {0U}, false};

static critical_section_t nmea_crit_sec;

/*============================================================================*/
/* SENSOR FUSION STATE                                                        */
/*============================================================================*/

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    bool initialized;
    uint32_t update_count;
} fusion_state_t;

static fusion_state_t fusion_state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0U};

/*============================================================================*/
/* INTER-CORE COMMUNICATION - DOUBLE BUFFERED                                 */
/*============================================================================*/

static TLM_serial_t tx_pool[TX_POOL_SIZE];
static TLM_serial_t rx_pool[RX_POOL_SIZE];
static volatile uint8_t tx_write_idx = 0U;
static volatile uint8_t rx_write_idx = 0U;

static system_health_t system_health = {
    .gps = HEALTH_NOMINAL,
    .mag = HEALTH_NOMINAL,
    .rtc = HEALTH_NOMINAL,
    .imu = HEALTH_NOMINAL,
    .fusion = HEALTH_NOMINAL,
    .crc_error_count = 0U,
    .sensor_timeout_count = 0U,
    .fusion_divergence_count = 0U
};

static uint32_t packet_sequence_core0 = 0U;

/*============================================================================*/
/* UTILITY FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Safe float comparison with epsilon
 */
static inline bool float_equals(float a, float b, float epsilon) {
    return fabsf(a - b) < epsilon;
}

/**
 * @brief Constrain value between min and max
 */
static inline float constrain_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief Wrap angle to [-180, 180] range
 */
static float wrap_180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief CRC-16-CCITT calculation (polynomial 0x1021)
 */
static uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFU;
    
    for (size_t i = 0U; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0U; j < 8U; j++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (crc << 1) ^ 0x1021U;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Verify packet integrity
 */
static bool verify_packet_integrity(const TLM_serial_t *packet) {
    if (packet->header != FIFO_HEADER_UINT16) return false;
    if (packet->footer != FIFO_FOOTER_UINT16) return false;
    
    uint16_t calc_crc = calculate_crc16((const uint8_t*)&packet->packet, 
                                         sizeof(TLM_packet_t));
    return (calc_crc == packet->crc16);
}

/*============================================================================*/
/* ICM42688 DUMMY FUNCTIONS (PLACEHOLDER)                                     */
/*============================================================================*/

/**
 * @brief ICM42688 initialization dummy
 * @note Replace with actual driver implementation
 */
static bool icm42688_init(i2c_inst_t *i2c) {
    (void)i2c;
    /* TODO: Implement actual ICM42688 initialization */
    return true;
}

/**
 * @brief Read ICM42688 raw data dummy
 * @note Replace with actual driver implementation
 */
static bool icm42688_read_raw(i2c_inst_t *i2c, icm42688_raw_t *data) {
    (void)i2c;
    
    /* Generate deterministic dummy data for testing */
    static uint32_t counter = 0U;
    counter++;
    
    /* Simulate 1g on Z-axis (hovering/stationary) */
    data->accel_x_raw = (int16_t)(100 * sinf(counter * 0.01f));
    data->accel_y_raw = (int16_t)(50 * cosf(counter * 0.02f));
    data->accel_z_raw = (int16_t)(16384); /* 1g at 16g range */
    
    /* Simulate small gyro drift */
    data->gyro_x_raw = (int16_t)(10 * sinf(counter * 0.005f));
    data->gyro_y_raw = (int16_t)(15 * cosf(counter * 0.007f));
    data->gyro_z_raw = (int16_t)(5 * sinf(counter * 0.003f));
    
    /* Simulate temperature around 25°C */
    data->temp_raw = (int16_t)(25 * 100);
    
    return true;
}

/**
 * @brief Scale ICM42688 raw data to engineering units
 */
static void icm42688_scale_data(const icm42688_raw_t *raw, icm42688_scaled_t *scaled) {
    scaled->accel_x_g = (float)raw->accel_x_raw * ICM42688_ACCEL_SCALE;
    scaled->accel_y_g = (float)raw->accel_y_raw * ICM42688_ACCEL_SCALE;
    scaled->accel_z_g = (float)raw->accel_z_raw * ICM42688_ACCEL_SCALE;
    
    scaled->gyro_x_dps = (float)raw->gyro_x_raw * ICM42688_GYRO_SCALE;
    scaled->gyro_y_dps = (float)raw->gyro_y_raw * ICM42688_GYRO_SCALE;
    scaled->gyro_z_dps = (float)raw->gyro_z_raw * ICM42688_GYRO_SCALE;
    
    scaled->temp_c = (float)raw->temp_raw / 100.0f;
}

/**
 * @brief Validate IMU sensor data
 */
static bool validate_imu_data(const icm42688_scaled_t *data) {
    if (fabsf(data->accel_x_g) > MAX_ACCEL_G) return false;
    if (fabsf(data->accel_y_g) > MAX_ACCEL_G) return false;
    if (fabsf(data->accel_z_g) > MAX_ACCEL_G) return false;
    
    if (fabsf(data->gyro_x_dps) > MAX_GYRO_DPS) return false;
    if (fabsf(data->gyro_y_dps) > MAX_GYRO_DPS) return false;
    if (fabsf(data->gyro_z_dps) > MAX_GYRO_DPS) return false;
    
    return true;
}

/*============================================================================*/
/* SENSOR FUSION - COMPLEMENTARY FILTER                                       */
/*============================================================================*/

/**
 * @brief Initialize sensor fusion with initial orientation
 */
static void fusion_initialize(const icm42688_scaled_t *imu, 
                              const qmc_5883_mag_read_t *mag) {
    /* Calculate initial roll and pitch from accelerometer */
    float ax = imu->accel_x_g;
    float ay = imu->accel_y_g;
    float az = imu->accel_z_g;
    
    fusion_state.roll = atan2f(ay, az) * 180.0f / M_PI;
    fusion_state.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    
    /* Calculate initial yaw from magnetometer */
    float mx = (float)mag->mag_x;
    float my = (float)mag->mag_y;
    float mz = (float)mag->mag_z;
    
    /* Tilt compensation for magnetometer */
    float cos_pitch = cosf(fusion_state.pitch * M_PI / 180.0f);
    float sin_pitch = sinf(fusion_state.pitch * M_PI / 180.0f);
    float cos_roll = cosf(fusion_state.roll * M_PI / 180.0f);
    float sin_roll = sinf(fusion_state.roll * M_PI / 180.0f);
    
    float mx_comp = mx * cos_pitch + mz * sin_pitch;
    float my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    
    fusion_state.yaw = atan2f(-my_comp, mx_comp) * 180.0f / M_PI;
    fusion_state.yaw = wrap_180(fusion_state.yaw + MAG_DECLINATION);
    
    fusion_state.gyro_bias_x = 0.0f;
    fusion_state.gyro_bias_y = 0.0f;
    fusion_state.gyro_bias_z = 0.0f;
    
    fusion_state.initialized = true;
    fusion_state.update_count = 0U;
}

/**
 * @brief Update attitude using complementary filter
 */
static void fusion_update(const icm42688_scaled_t *imu,
                         const qmc_5883_mag_read_t *mag,
                         attitude_data_t *attitude) {
    
    if (!fusion_state.initialized) {
        fusion_initialize(imu, mag);
        attitude->fusion_valid = false;
        return;
    }
    
    /* Apply gyro bias correction */
    float gx = imu->gyro_x_dps - fusion_state.gyro_bias_x;
    float gy = imu->gyro_y_dps - fusion_state.gyro_bias_y;
    float gz = imu->gyro_z_dps - fusion_state.gyro_bias_z;
    
    /* Gyro integration (prediction) */
    float roll_gyro = fusion_state.roll + gx * GYRO_DT;
    float pitch_gyro = fusion_state.pitch + gy * GYRO_DT;
    float yaw_gyro = fusion_state.yaw + gz * GYRO_DT;
    
    /* Accelerometer-based attitude (correction) */
    float ax = imu->accel_x_g;
    float ay = imu->accel_y_g;
    float az = imu->accel_z_g;
    
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    float roll_accel = atan2f(ay, az) * 180.0f / M_PI;
    float pitch_accel = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    
    /* Complementary filter fusion */
    if (accel_mag > 0.5f && accel_mag < 1.5f) {
        /* Valid accelerometer data (near 1g) */
        fusion_state.roll = COMPLEMENTARY_ALPHA * roll_gyro + 
                           (1.0f - COMPLEMENTARY_ALPHA) * roll_accel;
        fusion_state.pitch = COMPLEMENTARY_ALPHA * pitch_gyro + 
                            (1.0f - COMPLEMENTARY_ALPHA) * pitch_accel;
    } else {
        /* High acceleration - trust gyro only */
        fusion_state.roll = roll_gyro;
        fusion_state.pitch = pitch_gyro;
    }
    
    /* Magnetometer heading with tilt compensation */
    float mx = (float)mag->mag_x;
    float my = (float)mag->mag_y;
    float mz = (float)mag->mag_z;
    
    float cos_pitch = cosf(fusion_state.pitch * M_PI / 180.0f);
    float sin_pitch = sinf(fusion_state.pitch * M_PI / 180.0f);
    float cos_roll = cosf(fusion_state.roll * M_PI / 180.0f);
    float sin_roll = sinf(fusion_state.roll * M_PI / 180.0f);
    
    float mx_comp = mx * cos_pitch + mz * sin_pitch;
    float my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    
    float heading_mag = atan2f(-my_comp, mx_comp) * 180.0f / M_PI;
    heading_mag = wrap_180(heading_mag + MAG_DECLINATION);
    
    /* Yaw fusion (slower response for magnetometer) */
    const float MAG_WEIGHT = 0.05f;
    float yaw_diff = wrap_180(heading_mag - fusion_state.yaw);
    fusion_state.yaw = wrap_180(fusion_state.yaw + MAG_WEIGHT * yaw_diff);
    
    /* Populate output */
    attitude->roll_deg = fusion_state.roll;
    attitude->pitch_deg = fusion_state.pitch;
    attitude->yaw_deg = fusion_state.yaw;
    attitude->roll_rate_dps = gx;
    attitude->pitch_rate_dps = gy;
    attitude->yaw_rate_dps = gz;
    attitude->accel_magnitude_g = accel_mag;
    attitude->heading_mag_deg = heading_mag;
    attitude->fusion_valid = true;
    
    fusion_state.update_count++;
}

/*============================================================================*/
/* HEALTH MONITORING                                                          */
/*============================================================================*/

/**
 * @brief Update system health status
 */
static void update_health_status(TLM_packet_t *packet) {
    absolute_time_t now = get_absolute_time();
    
    /* Check GPS health */
    int64_t gps_age_ms = absolute_time_diff_us(system_health.last_gps_update, now) / 1000;
    if (gps_age_ms > SENSOR_TIMEOUT_MS) {
        system_health.gps = HEALTH_DEGRADED;
    } else if (packet->gnrmc.status == 'A') {
        system_health.gps = HEALTH_NOMINAL;
    }
    
    /* Check IMU health */
    int64_t imu_age_ms = absolute_time_diff_us(system_health.last_imu_update, now) / 1000;
    if (imu_age_ms > SENSOR_TIMEOUT_MS) {
        system_health.imu = HEALTH_FAILED;
    } else {
        system_health.imu = HEALTH_NOMINAL;
    }
    
    /* Check magnetometer health */
    int64_t mag_age_ms = absolute_time_diff_us(system_health.last_mag_update, now) / 1000;
    if (mag_age_ms > SENSOR_TIMEOUT_MS) {
        system_health.mag = HEALTH_DEGRADED;
    } else {
        system_health.mag = HEALTH_NOMINAL;
    }
    
    /* Check fusion health */
    if (packet->attitude.fusion_valid) {
        system_health.fusion = HEALTH_NOMINAL;
    } else {
        system_health.fusion = HEALTH_DEGRADED;
    }
    
    /* Check CRC error rate */
    if (system_health.crc_error_count > MAX_CRC_ERRORS) {
        /* Critical error - reset system */
        watchdog_enable(1, true);
        while(1); /* Force watchdog reset */
    }
    
    /* Copy health to packet */
    memcpy(&packet->health, &system_health, sizeof(system_health_t));
}

/*============================================================================*/
/* HARDWARE INITIALIZATION                                                    */
/*============================================================================*/

static inline void init_i2c(void) {
    i2c_init(I2C_PORT, 400000U);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void __isr __time_critical_func(uart0_irq_handler)(void) {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);

        if (!nmea_state.receiving) {
            bool prefix_started = false;

            for (int i = 0; i < (int)NMEA_NUM_PREFIXES; i++) {
                uint8_t pos = nmea_state.prefix_pos[i];
                
                if (ch == (uint8_t)nmea_prefixes[i][pos]) {
                    nmea_state.prefix_pos[i]++;
                    prefix_started = true;

                    if (nmea_state.prefix_pos[i] == prefix_lengths[i]) {
                        nmea_state.matched_prefix = (int8_t)i;
                        nmea_state.receiving = true;
                        nmea_write_idx = 0U;
                        
                        memcpy((void*)nmea_buffers[write_buffer_idx], 
                               nmea_prefixes[i], prefix_lengths[i]);
                        nmea_write_idx = prefix_lengths[i];
                        memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
                        break;
                    }
                } else {
                    nmea_state.prefix_pos[i] = (ch == '$') ? 1U : 0U;
                    if (ch == '$') {
                        prefix_started = true;
                    }
                }
            }

            if (!prefix_started) {
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }

        } else {
            if (nmea_write_idx < (NMEA_BUFFER_SIZE - 1U)) {
                nmea_buffers[write_buffer_idx][nmea_write_idx++] = ch;

                if (ch == '\n') {
                    nmea_buffers[write_buffer_idx][nmea_write_idx] = '\0';
                    ready_buffer_idx = (int8_t)write_buffer_idx;
                    write_buffer_idx ^= 1U;
                    nmea_data_ready = true;
                    nmea_state.receiving = false;
                    nmea_state.matched_prefix = -1;
                    system_health.last_gps_update = get_absolute_time();
                }
            } else {
                nmea_state.receiving = false;
                nmea_write_idx = 0U;
                nmea_state.matched_prefix = -1;
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }
        }
    }
}

static TLM_serial_t core1_received_packet;
static volatile bool core1_data_available = false;

void core1_sio_irq(void) {
    while (multicore_fifo_rvalid()) {
        uint32_t src_ptr = multicore_fifo_pop_blocking();
        TLM_serial_t *src = (TLM_serial_t *)src_ptr;
        memcpy(&core1_received_packet, src, sizeof(TLM_serial_t));
        core1_data_available = true;
    }
    multicore_fifo_clear_irq();
}

static inline void init_uart(void) {
    uart_init(uart0, NEOM8N_UART_BAUDRATE);
    gpio_set_function(NEOM8N_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NEOM8N_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8U, 1U, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    hw_write_masked(&uart_get_hw(uart0)->ifls,
                    2U << UART_UARTIFLS_RXIFLSEL_LSB,
                    UART_UARTIFLS_RXIFLSEL_BITS);

    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
    irq_set_priority(UART0_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
}

/*============================================================================*/
/* TELEMETRY FUNCTIONS - CORE 0                                               */
/*============================================================================*/

void telemetry_acquire_sensor(TLM_packet_t* tlm_packet, current_device_t device) {
    switch(device) {
        case DEV_GPS:
            if (nmea_data_ready) {
                critical_section_enter_blocking(&nmea_crit_sec);
                if (ready_buffer_idx >= 0) {
                    nmea_parse_sentence((char*)nmea_buffers[ready_buffer_idx], 
                                      &tlm_packet->gnrmc, 
                                      &tlm_packet->gngga, 
                                      &tlm_packet->gnvtg);
                    ready_buffer_idx = -1;
                }
                nmea_data_ready = false;
                critical_section_exit(&nmea_crit_sec);
            }
            break;

        case DEV_MAG:
            if (qmc5883l_read_mag_drdy(I2C_PORT, &tlm_packet->mag)) {
                system_health.last_mag_update = get_absolute_time();
            }
            break;

        case DEV_RTC:
            ds3231_read_time(I2C_PORT, &tlm_packet->timeframe);
            break;

        case DEV_IMU:
            if (icm42688_read_raw(I2C_PORT, &tlm_packet->imu_raw)) {
                icm42688_scale_data(&tlm_packet->imu_raw, &tlm_packet->imu_scaled);
                if (validate_imu_data(&tlm_packet->imu_scaled)) {
                    system_health.last_imu_update = get_absolute_time();
                }
            }
            break;

        case DEV_FUSION:
            fusion_update(&tlm_packet->imu_scaled, 
                         &tlm_packet->mag, 
                         &tlm_packet->attitude);
            break;

        default:
            break;
    }
}

void tlm_serialize_packet(TLM_packet_t* tlm_packet, TLM_serial_t* tlm_serial) {
    tlm_serial->header = FIFO_HEADER_UINT16;
    memcpy(&tlm_serial->packet, tlm_packet, sizeof(TLM_packet_t));
    
    tlm_serial->crc16 = calculate_crc16((const uint8_t*)&tlm_serial->packet, 
                                         sizeof(TLM_packet_t));
    tlm_serial->footer = FIFO_FOOTER_UINT16;
}

/**
 * @brief Queue telemetry to Core 1 with overflow protection
 */
static bool queue_tlm_to_core1(TLM_serial_t *src) {
    uint8_t idx = tx_write_idx % TX_POOL_SIZE;
    tx_write_idx++;
    
    memcpy(&tx_pool[idx], src, sizeof(TLM_serial_t));
    
    /* Non-blocking push with timeout */
    uint32_t timeout = time_us_32() + 10000U; /* 10ms timeout */
    while (!multicore_fifo_wready()) {
        if (time_us_32() > timeout) {
            return false; /* FIFO full */
        }
        tight_loop_contents();
    }
    
    multicore_fifo_push_blocking((uint32_t)&tx_pool[idx]);
    return true;
}

/**
 * @brief Receive telemetry from Core 1 (non-blocking with timeout)
 */
static bool receive_tlm_from_core1(TLM_serial_t *dest, uint32_t timeout_us) {
    uint32_t start = time_us_32();
    
    while (!multicore_fifo_rvalid()) {
        if ((time_us_32() - start) > timeout_us) {
            return false;
        }
        tight_loop_contents();
    }
    
    TLM_serial_t *src = (TLM_serial_t *)multicore_fifo_pop_blocking();
    memcpy(dest, src, sizeof(TLM_serial_t));
    
    return verify_packet_integrity(dest);
}

/*============================================================================*/
/* CORE 1 ENTRY POINT - TELEMETRY OUTPUT                                     */
/*============================================================================*/

void core1_entry(void) {
    /* Setup inter-core interrupt */
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_sio_irq);
    irq_set_enabled(SIO_IRQ_PROC1, true);

    uint32_t packet_count = 0U;
    uint32_t valid_packets = 0U;
    uint32_t invalid_packets = 0U;
    absolute_time_t last_print_time = get_absolute_time();
    
    printf("[CORE1] Telemetry processor started\n");
    
    while (true) {
        if (core1_data_available) {
            core1_data_available = false;
            packet_count++;
            
            /* Verify packet integrity */
            bool header_valid = (core1_received_packet.header == FIFO_HEADER_UINT16);
            bool footer_valid = (core1_received_packet.footer == FIFO_FOOTER_UINT16);
            bool crc_valid = verify_packet_integrity(&core1_received_packet);
            
            if (header_valid && footer_valid && crc_valid) {
                valid_packets++;
                
                /* Print telemetry at reduced rate (1Hz) */
                absolute_time_t now = get_absolute_time();
                int64_t dt_ms = absolute_time_diff_us(last_print_time, now) / 1000;
                
                if (dt_ms >= 1000) {
                    last_print_time = now;
                    
                    printf("-- TELEMETRY PACKET #%-5lu | Core1 Uptime: %llu ms         --\n", 
                           core1_received_packet.packet.packet_sequence,
                           to_ms_since_boot(now));
                    
                    /* RTC Timestamp */
                    printf("-- [RTC] %04d-%02d-%02d %02d:%02d:%02d %-12s           --\n",
                           core1_received_packet.packet.timeframe.time.year,
                           core1_received_packet.packet.timeframe.time.month,
                           core1_received_packet.packet.timeframe.time.day,
                           core1_received_packet.packet.timeframe.time.hour,
                           core1_received_packet.packet.timeframe.time.minute,
                           core1_received_packet.packet.timeframe.time.second,
                           core1_received_packet.packet.timeframe.time.is_time_synced ? 
                           "[SYNCED]" : "[NO SYNC]");
                    
                    
                    /* IMU Data - Raw */
                    printf("-- [IMU RAW] Ax:%6d Ay:%6d Az:%6d                  --\n",
                           core1_received_packet.packet.imu_raw.accel_x_raw,
                           core1_received_packet.packet.imu_raw.accel_y_raw,
                           core1_received_packet.packet.imu_raw.accel_z_raw);
                    printf("--           Gx:%6d Gy:%6d Gz:%6d Temp:%6d°C  --\n",
                           core1_received_packet.packet.imu_raw.gyro_x_raw,
                           core1_received_packet.packet.imu_raw.gyro_y_raw,
                           core1_received_packet.packet.imu_raw.gyro_z_raw,
                           core1_received_packet.packet.imu_raw.temp_raw);
                    
                    
                    /* IMU Data - Scaled */
                    printf("-- [IMU]     Ax:%7.3fg Ay:%7.3fg Az:%7.3fg         --\n",
                           core1_received_packet.packet.imu_scaled.accel_x_g,
                           core1_received_packet.packet.imu_scaled.accel_y_g,
                           core1_received_packet.packet.imu_scaled.accel_z_g);
                    printf("--           Gx:%7.2f° Gy:%7.2f° Gz:%7.2f° T:%5.1f°C --\n",
                           core1_received_packet.packet.imu_scaled.gyro_x_dps,
                           core1_received_packet.packet.imu_scaled.gyro_y_dps,
                           core1_received_packet.packet.imu_scaled.gyro_z_dps,
                           core1_received_packet.packet.imu_scaled.temp_c);
                    
                    
                    /* Attitude (Fused) */
                    printf("-- [ATTITUDE] Roll:%7.2f° Pitch:%7.2f° Yaw:%7.2f°   --\n",
                           core1_received_packet.packet.attitude.roll_deg,
                           core1_received_packet.packet.attitude.pitch_deg,
                           core1_received_packet.packet.attitude.yaw_deg);
                    printf("--            Rate: R:%6.2f P:%6.2f Y:%6.2f °/s      --\n",
                           core1_received_packet.packet.attitude.roll_rate_dps,
                           core1_received_packet.packet.attitude.pitch_rate_dps,
                           core1_received_packet.packet.attitude.yaw_rate_dps);
                    printf("--            |Accel|:%5.3fg Mag Hdg:%7.2f° %s      --\n",
                           core1_received_packet.packet.attitude.accel_magnitude_g,
                           core1_received_packet.packet.attitude.heading_mag_deg,
                           core1_received_packet.packet.attitude.fusion_valid ? 
                           "[VALID]" : "[INIT]");
                    
                    
                    /* Magnetometer */
                    printf("-- [MAG]     X:%6d Y:%6d Z:%6d (counts)           --\n",
                           core1_received_packet.packet.mag.mag_x,
                           core1_received_packet.packet.mag.mag_y,
                           core1_received_packet.packet.mag.mag_z);
                    
                    
                    /* GPS Data */
                    printf("-- [GPS] Status:%c Time:%s Date:%s                  --\n",
                           core1_received_packet.packet.gnrmc.status,
                           core1_received_packet.packet.gnrmc.utc_time,
                           core1_received_packet.packet.gnrmc.date);
                    printf("--       Lat:%s%c Lon:%s%c                           --\n",
                           core1_received_packet.packet.gnrmc.lat,
                           core1_received_packet.packet.gnrmc.ns,
                           core1_received_packet.packet.gnrmc.lon,
                           core1_received_packet.packet.gnrmc.ew);
                    printf("--       Speed:%6.2fkts Course:%6.2f° Fix:%d Sats:%2d    --\n",
                           core1_received_packet.packet.gnrmc.speed_knots,
                           core1_received_packet.packet.gnrmc.course_deg,
                           core1_received_packet.packet.gngga.fix_quality,
                           core1_received_packet.packet.gngga.num_satellites);
                    printf("--       Alt:%7.2fm HDOP:%5.2f                            --\n",
                           core1_received_packet.packet.gngga.altitude,
                           core1_received_packet.packet.gngga.hdop);
                    
                    
                    /* System Health */
                    const char* health_str[] = {"NOM", "DEG", "CRT", "FAIL"};
                    printf("-- [HEALTH] GPS:%-4s MAG:%-4s RTC:%-4s IMU:%-4s FUS:%-4s --\n",
                           health_str[core1_received_packet.packet.health.gps],
                           health_str[core1_received_packet.packet.health.mag],
                           health_str[core1_received_packet.packet.health.rtc],
                           health_str[core1_received_packet.packet.health.imu],
                           health_str[core1_received_packet.packet.health.fusion]);
                    printf("-- [ERRORS] CRC:%3lu Timeout:%3lu Fusion:%3lu             --\n",
                           core1_received_packet.packet.health.crc_error_count,
                           core1_received_packet.packet.health.sensor_timeout_count,
                           core1_received_packet.packet.health.fusion_divergence_count);
                    
                    
                    /* Statistics */
                    float packet_success_rate = (packet_count > 0U) ? 
                                               (100.0f * (float)valid_packets / (float)packet_count) : 0.0f;
                    printf("-- [STATS] Valid:%lu Invalid:%lu Success:%.1f%%            --\n",
                           valid_packets, invalid_packets, packet_success_rate);
                }
                
                /* Echo packet back to Core 0 for acknowledgment */
                uint8_t idx = rx_write_idx % RX_POOL_SIZE;
                rx_write_idx++;
                memcpy(&rx_pool[idx], &core1_received_packet, sizeof(TLM_serial_t));
                multicore_fifo_push_blocking((uint32_t)&rx_pool[idx]);
                
            } else {
                invalid_packets++;
                system_health.crc_error_count++;
                
                printf("[CORE1 ERROR] Packet validation failed: Header:%d Footer:%d CRC:%d\n\n\n\n",
                       header_valid, footer_valid, crc_valid);
            }
        }
        
        tight_loop_contents();
    }
}

/*============================================================================*/
/* MAIN APPLICATION - CORE 0                                                  */
/*============================================================================*/

int main(void) {
    /* Initialize USB serial FIRST - critical for debug */
    stdio_init_all();
    sleep_ms(2000U);
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("--     GNC TEST BENCH TELEMETRY SYSTEM v2.0.0                --\n");
    printf("--     Aerospace Compliant Implementation                     --\n");
    printf("--     DO-178C Level C | MISRA-C:2012                         --\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    /* Initialize critical section for NMEA parsing */
    critical_section_init(&nmea_crit_sec);
    printf("[INIT] Critical sections initialized\n");
    
    /* Launch Core 1 BEFORE enabling watchdog */
    multicore_launch_core1(core1_entry);
    printf("[INIT] Core 1 launched - Telemetry processor active\n");
    sleep_ms(100U);
    
    /* Enable watchdog with 8 second timeout */
    if (watchdog_caused_reboot()) {
        printf("[WARN] System recovered from watchdog reset\n");
    }
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    printf("[INIT] Watchdog enabled (%dms timeout)\n", WATCHDOG_TIMEOUT_MS);
    
    /* Initialize hardware peripherals */
    init_i2c();
    printf("[INIT] I2C initialized (400kHz)\n");
    
    init_uart();
    printf("[INIT] UART initialized (GPS @ %dbps)\n", NEOM8N_UART_BAUDRATE);
    
    /* Initialize sensors */
    if (qmc_5883l_init(I2C_PORT)) {
        printf("[INIT] QMC5883L magnetometer initialized\n");
    } else {
        printf("[ERROR] QMC5883L initialization failed\n");
        system_health.mag = HEALTH_FAILED;
    }
    
    if (ds3231_init(I2C_PORT)) {
        printf("[INIT] DS3231 RTC initialized\n");
    } else {
        printf("[ERROR] DS3231 initialization failed\n");
        system_health.rtc = HEALTH_FAILED;
    }
    
    if (mpu6050_init(I2C_PORT)) {
        printf("[INIT] MPU6050 (backup IMU) initialized\n");
    }
    
    if (icm42688_init(I2C_PORT)) {
        printf("[INIT] ICM42688 IMU initialized (DUMMY)\n");
    } else {
        printf("[WARN] ICM42688 initialization failed (using dummy data)\n");
    }
    
    printf("\n[READY] System operational - entering main telemetry loop\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    /* Initialize telemetry structures */
    TLM_packet_t tlm_packet = {0};
    TLM_serial_t tlm_serial = {0};
    TLM_serial_t ack_packet = {0};
    
    /* Initialize health timestamps */
    system_health.last_gps_update = get_absolute_time();
    system_health.last_mag_update = get_absolute_time();
    system_health.last_imu_update = get_absolute_time();
    
    current_device_t current_device = DEV_IMU;
    uint32_t loop_count = 0U;
    absolute_time_t cycle_start;
    
    /* Main telemetry loop - 100Hz (10ms cycle) */
    while (true) {
        cycle_start = get_absolute_time();
        watchdog_update();
        
        /* Round-robin sensor acquisition */
        telemetry_acquire_sensor(&tlm_packet, current_device);
        current_device = (current_device_t)((current_device + 1U) % DEV_COUNT);
        
        /* Every 10th iteration, update health and serialize */
        if ((loop_count % 10U) == 0U) {
            /* Update system health status */
            update_health_status(&tlm_packet);
            
            /* Add metadata */
            tlm_packet.packet_sequence = packet_sequence_core0++;
            tlm_packet.core0_timestamp_ms = to_ms_since_boot(get_absolute_time());
            
            /* Serialize and send to Core 1 */
            tlm_serialize_packet(&tlm_packet, &tlm_serial);
            
            if (!queue_tlm_to_core1(&tlm_serial)) {
                printf("[CORE0 WARN] FIFO overflow - Core 1 slow?\n");
            }
            
            /* Wait for acknowledgment from Core 1 (with timeout) */
            if (!receive_tlm_from_core1(&ack_packet, 50000U)) {
                printf("[CORE0 WARN] Core 1 ACK timeout\n");
                system_health.sensor_timeout_count++;
            }
        }
        
        loop_count++;
        
        /* Maintain 10ms cycle time */
        absolute_time_t cycle_end = get_absolute_time();
        int64_t elapsed_us = absolute_time_diff_us(cycle_start, cycle_end);
        int64_t sleep_us_num = 10000 - elapsed_us;
        
        if (sleep_us > 0) {
            sleep_us(sleep_us_num);
        } else if ((loop_count % 100U) == 0U) {
            printf("[CORE0 WARN] Cycle overrun: %lldus (target: 10000us)\n", elapsed_us);
        }
    }
    return 0;
} 