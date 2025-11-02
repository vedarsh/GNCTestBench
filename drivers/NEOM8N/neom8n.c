#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "neom8n.h"

/*
split_fields: Helper function to split NMEA sentence into fields
Returns number of fields found, up to max_fields.
Assumes fields array has space for max_fields pointers.

TODO: handle edge cases like empty fields, checksum, etc.
*/

static int split_fields(char *sentence, char *fields[], int max_fields) {
    int count = 0;
    char *ptr = sentence;
    while (*ptr && count < max_fields) {
        fields[count++] = ptr;
        while (*ptr && *ptr != ',') ptr++;
        if (*ptr == ',') *ptr++ = '\0';
    }
    return count;
}

/*
nmea_parse_sentence: Parse an NMEA sentence and populate the appropriate structure.
Returns the type of sentence parsed, or NMEA_TYPE_UNKNOWN if unrecognized or error.
Parses the following sentence types:
- GNRMC: Recommended Minimum Specific GNSS Data
- GNGGA: Global Positioning System Fix Data
- GNVTG: Course Over Ground and Ground Speed
*/

nmea_type_t nmea_parse_sentence(char* sentence,
                                nmea_gnrmc_t* rmc,
                                nmea_gngga_t* gga,
                                nmea_gnvtg_t* vtg)
{
    if (!sentence) return NMEA_TYPE_UNKNOWN;

    // Determine the sentence type
    nmea_type_t type = NMEA_TYPE_UNKNOWN;
    if (strstr(sentence, "$GNRMC")) type = NMEA_TYPE_GNRMC;
    else if (strstr(sentence, "$GNGGA")) type = NMEA_TYPE_GNGGA;
    else if (strstr(sentence, "$GNVTG")) type = NMEA_TYPE_GNVTG;
    else return NMEA_TYPE_UNKNOWN;

    // Make a local copy to modify safely
    char buf[NMEA_MAX_SENTENCE_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[20] = {0};
    int n = split_fields(buf, fields, 20);

    switch (type)
    {
        case NMEA_TYPE_GNRMC:
            if (n < 10 || !rmc) break;
            snprintf(rmc->utc_time, sizeof(rmc->utc_time), "%s", fields[1]);
            rmc->status = fields[2][0];
            snprintf(rmc->lat, sizeof(rmc->lat), "%s", fields[3]);
            rmc->ns = fields[4][0];
            snprintf(rmc->lon, sizeof(rmc->lon), "%s", fields[5]);
            rmc->ew = fields[6][0];
            rmc->speed_knots = fields[7][0] ? atof(fields[7]) : 0.0f;
            rmc->course_deg = fields[8][0] ? atof(fields[8]) : 0.0f;
            snprintf(rmc->date, sizeof(rmc->date), "%s", fields[9]);
            break;

        case NMEA_TYPE_GNGGA:
            if (n < 10 || !gga) break;
            snprintf(gga->utc_time, sizeof(gga->utc_time), "%s", fields[1]);
            gga->lat = fields[2][0] ? atof(fields[2]) : 0.0;
            gga->ns = fields[3][0];
            gga->lon = fields[4][0] ? atof(fields[4]) : 0.0;
            gga->ew = fields[5][0];
            gga->fix_quality = fields[6][0] ? atoi(fields[6]) : 0;
            gga->num_satellites = fields[7][0] ? atoi(fields[7]) : 0;
            gga->hdop = fields[8][0] ? atof(fields[8]) : 0.0f;
            gga->altitude = fields[9][0] ? atof(fields[9]) : 0.0f;
            break;

        case NMEA_TYPE_GNVTG:
            if (n < 9 || !vtg) break;
            vtg->course_true = fields[1][0] ? atof(fields[1]) : 0.0f;
            vtg->course_magnetic = fields[3][0] ? atof(fields[3]) : 0.0f;
            vtg->speed_knots = fields[5][0] ? atof(fields[5]) : 0.0f;
            vtg->speed_kmh = fields[7][0] ? atof(fields[7]) : 0.0f;
            break;

        default:
            break;
    }

    return type;
}

/*============================================================================*/
/* GPS TIME SYNCHRONIZATION                                                   */
/*============================================================================*/

/**
 * @brief Parse GPS time string to milliseconds since GPS epoch
 * @param[in] utc_time UTC time string (HHMMSS.sss)
 * @param[in] date Date string (DDMMYY)
 * @return Milliseconds since GPS epoch (Jan 6, 1980)
 */
static uint32_t gps_time_to_epoch_ms(const char *utc_time, const char *date) {
    if (strlen(utc_time) < 6U || strlen(date) < 6U) {
        return 0U;
    }
    
    /* Parse time components */
    uint32_t hours = (uint32_t)((utc_time[0] - '0') * 10U + (utc_time[1] - '0'));
    uint32_t minutes = (uint32_t)((utc_time[2] - '0') * 10U + (utc_time[3] - '0'));
    uint32_t seconds = (uint32_t)((utc_time[4] - '0') * 10U + (utc_time[5] - '0'));
    
    /* Parse date components */
    uint32_t day = (uint32_t)((date[0] - '0') * 10U + (date[1] - '0'));
    uint32_t month = (uint32_t)((date[2] - '0') * 10U + (date[3] - '0'));
    uint32_t year = (uint32_t)((date[4] - '0') * 10U + (date[5] - '0')) + 2000U;
    
    /* Simplified epoch calculation (approximation) */
    /* Days since GPS epoch (Jan 6, 1980) */
    uint32_t days_since_epoch = (year - 1980U) * 365U + (year - 1980U) / 4U;
    
    /* Add days for months */
    const uint32_t days_per_month[] = {31U, 28U, 31U, 30U, 31U, 30U, 
                                       31U, 31U, 30U, 31U, 30U, 31U};
    for (uint32_t m = 1U; m < month; m++) {
        days_since_epoch += days_per_month[m - 1U];
    }
    days_since_epoch += day;
    
    /* Convert to milliseconds */
    uint32_t epoch_ms = (days_since_epoch * 86400U + hours * 3600U + 
                         minutes * 60U + seconds) * 1000U;
    
    return epoch_ms;
}

