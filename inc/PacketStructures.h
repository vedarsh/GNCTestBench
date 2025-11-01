#ifndef __PACKETSTRUCTURES_H__
#define __PACKETSTRUCTURES_H__

#include "main.h"

/*============================================================================*/
/* CONFIGURATION PARAMETERS                                                   */
/*============================================================================*/

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
#define MAG_DECLINATION 0.0f  /* TODO - Adjust for local magnetic declination */
#define COMPLEMENTARY_ALPHA 0.98f

/* Fault detection thresholds */
#define MAX_ACCEL_G 16.0f
#define MAX_GYRO_DPS 2000.0f
#define MAX_MAG_GAUSS 8.0f
#define SENSOR_TIMEOUT_MS 500U
#define MAX_CRC_ERRORS 10U
#define MAX_CONSECUTIVE_FAULTS 5U

/* ICM42688 dummy conversion factors (TODO - will be replaced with actual driver) */
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

typedef struct {
    bool is_time_synced;

    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
}timeframe_t;

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
    timeframe_t timeframe;
    nmea_gnrmc_t gnrmc;
    nmea_gngga_t gngga;
    nmea_gnvtg_t gnvtg;
    icm42686_scaled_t imu_scaled;
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

#endif