#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NMEA_MAX_SENTENCE_LEN 256
typedef struct {
    char utc_time[16];
    char lat[16];
    char ns;
    char lon[16];
    char ew;
    uint16_t speed_knots;   /* Speed * 100 (e.g., 1234 = 12.34 knots) */
    uint16_t course_deg;    /* Course * 100 (e.g., 27550 = 275.50°) */
    char date[8];
    char status;
} nmea_gnrmc_t;

typedef struct {
    char utc_time[16];
    int32_t lat;            /* Latitude * 10000000 */
    char ns;
    int32_t lon;            /* Longitude * 10000000 */
    char ew;
    int fix_quality;
    int num_satellites;
    int32_t hdop;           /* HDOP * 100 */
    int32_t altitude;       /* Altitude * 100 (cm) */
} nmea_gngga_t;

typedef struct {
    int32_t course_true;     /* Course * 100 */
    int32_t course_magnetic; /* Course * 100 */
    int32_t speed_knots;     /* Speed * 100 */
    int32_t speed_kmh;       /* Speed * 100 */
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
