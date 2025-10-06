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
