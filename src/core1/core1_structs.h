#ifndef __CORE1_STRUCTS_H__
#define __CORE1_STRUCTS_H__

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    char utc_time[11];
    char status;
    char lat[10];
    char ns;
    char lon[11];
    char ew;
    float speed_knots;
    float course_deg;
    char date[7];
} nmea_gnrmc_tlm_t;

typedef struct {
    char utc_time[11];
    double lat;
    char ns;
    double lon;
    char ew;
    int fix_quality;
    int num_satellites;
    float hdop;
    float altitude;
} nmea_gngga_tlm_t;

typedef struct {
    float course_true;
    float course_magnetic;
    float speed_knots;
    float speed_kmh;
} nmea_gnvtg_tlm_t;

typedef union {
    struct {
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;
    };
    int16_t raw[3];  // same 6 bytes accessible as array
} qmc_5883_mag_read_tlm_t;

#pragma pack(push,1)
typedef struct {
    qmc_5883_mag_read_tlm_t mag;
    nmea_gnrmc_tlm_t gnrmc;
    nmea_gngga_tlm_t gngga;
    nmea_gnvtg_tlm_t gnvtg;
} TLM_packet_rcv_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct TLM_serial {
    uint16_t header;
    TLM_packet_rcv_t packet;
    uint16_t CRC16;
} TLM_serial_rcv_t;
#pragma pack(pop)

#endif