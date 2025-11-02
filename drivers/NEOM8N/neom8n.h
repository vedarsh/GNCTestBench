#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NMEA_MAX_SENTENCE_LEN 256

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
} nmea_gnrmc_t;

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
} nmea_gngga_t;

typedef struct {
    float course_true;
    float course_magnetic;
    float speed_knots;
    float speed_kmh;
} nmea_gnvtg_t;

// Enum for sentence types
typedef enum {
    NMEA_TYPE_UNKNOWN = 0,
    NMEA_TYPE_GNRMC,
    NMEA_TYPE_GNGGA,
    NMEA_TYPE_GNVTG
} nmea_type_t;

// Parser function
nmea_type_t nmea_parse_sentence(char* sentence,
                                nmea_gnrmc_t* rmc,
                                nmea_gngga_t* gga,
                                nmea_gnvtg_t* vtg);


static uint32_t gps_time_to_epoch_ms(const char *utc_time, const char *date);
