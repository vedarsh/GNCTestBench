#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "neom8n.h"

/*
 * Aerospace-grade NMEA parser - Integer-only version for RP2040
 * No floating point operations - uses fixed-point arithmetic
 */

/* Constants */
#define MAX_FIELD_COUNT (20U)
#define MAX_SENTENCE_LEN (256U)
#define MAX_FIELD_LEN (32U)
#define MAX_STRING_ITER (256U)

/* Fixed-point scaling factors */
#define SPEED_SCALE (100)      /* 2 decimal places for speed */
#define COURSE_SCALE (100)     /* 2 decimal places for course */
#define HDOP_SCALE (100)       /* 2 decimal places for HDOP */
#define ALT_SCALE (100)        /* 2 decimal places for altitude */
#define COORD_SCALE (10000000) /* 7 decimal places for coordinates */

/* Return codes */
typedef enum {
    PARSE_OK = 0,
    PARSE_ERR_NULL = 1,
    PARSE_ERR_BOUNDS = 2,
    PARSE_ERR_FORMAT = 3,
    PARSE_ERR_INVALID = 4
} parse_result_t;

/*
 * is_valid_digit: Check if character is a digit
 */
static uint8_t is_valid_digit(char c)
{
    uint8_t result = 0U;
    if ((c >= '0') && (c <= '9')) {
        result = 1U;
    }
    return result;
}

/*
 * is_valid_number_char: Check if character is valid in a number
 */
static uint8_t is_valid_number_char(char c)
{
    uint8_t result = 0U;
    if (is_valid_digit(c) != 0U) {
        result = 1U;
    } else if ((c == '.') || (c == '-') || (c == '+')) {
        result = 1U;
    } else {
        result = 0U;
    }
    return result;
}

/*
 * safe_strlen: Bounded string length calculation
 */
static uint32_t safe_strlen(const char *str, uint32_t max_len)
{
    uint32_t len = 0U;
    uint32_t i = 0U;
    
    if (str == NULL) {
        return 0U;
    }
    
    for (i = 0U; i < max_len; i++) {
        if (str[i] == '\0') {
            break;
        }
        len++;
    }
    
    return len;
}

/*
 * parse_fixed_int: Parse decimal number to fixed-point integer
 * Example: "12.34" with scale=100 -> 1234
 */
static parse_result_t parse_fixed_int(const char *str, int32_t *result, uint32_t scale)
{
    uint32_t i = 0U;
    uint32_t len = 0U;
    uint8_t has_digit = 0U;
    uint8_t has_dot = 0U;
    uint32_t start_idx = 0U;
    
    /* Validate inputs */
    if ((str == NULL) || (result == NULL)) {
        return PARSE_ERR_NULL;
    }
    
    /* Initialize result */
    *result = 0;
    
    /* Check string length */
    len = safe_strlen(str, MAX_FIELD_LEN);
    if (len == 0U) {
        return PARSE_OK; /* Empty string = 0 */
    }
    
    /* Skip leading sign */
    if ((str[0] == '+') || (str[0] == '-')) {
        start_idx = 1U;
    }
    
    /* Validate all characters */
    for (i = start_idx; i < len; i++) {
        if (is_valid_digit(str[i]) != 0U) {
            has_digit = 1U;
        } else if (str[i] == '.') {
            if (has_dot != 0U) {
                return PARSE_ERR_FORMAT; /* Multiple dots */
            }
            has_dot = 1U;
        } else {
            return PARSE_ERR_FORMAT; /* Invalid character */
        }
    }
    
    /* Must have at least one digit */
    if (has_digit == 0U) {
        return PARSE_ERR_FORMAT;
    }
    
    /* Manual parsing to fixed-point integer */
    {
        int32_t int_part = 0;
        int32_t frac_part = 0;
        uint32_t frac_digits = 0U;
        uint8_t is_negative = 0U;
        uint8_t past_decimal = 0U;
        uint32_t scale_divisor = 1U;
        
        if (str[0] == '-') {
            is_negative = 1U;
            i = 1U;
        } else if (str[0] == '+') {
            i = 1U;
        } else {
            i = 0U;
        }
        
        /* Parse integer and fractional parts separately */
        for (; i < len; i++) {
            if (str[i] == '.') {
                past_decimal = 1U;
            } else if (is_valid_digit(str[i]) != 0U) {
                uint8_t digit = (uint8_t)(str[i] - '0');
                
                if (past_decimal == 0U) {
                    int_part = (int_part * 10) + (int32_t)digit;
                } else {
                    frac_part = (frac_part * 10) + (int32_t)digit;
                    frac_digits++;
                }
            }
        }
        
        /* Calculate scale divisor based on fractional digits */
        for (i = 0U; i < frac_digits; i++) {
            scale_divisor *= 10U;
        }
        
        /* Convert to fixed-point: (int_part * scale) + (frac_part * scale / scale_divisor) */
        int32_t value = (int_part * (int32_t)scale);
        if (frac_digits > 0U) {
            value += ((frac_part * (int32_t)scale) / (int32_t)scale_divisor);
        }
        
        if (is_negative != 0U) {
            value = -value;
        }
        
        *result = value;
    }
    
    return PARSE_OK;
}

/*
 * parse_int: Safe integer parsing with full validation
 */
static parse_result_t parse_int(const char *str, int32_t *result)
{
    uint32_t i = 0U;
    uint32_t len = 0U;
    uint8_t has_digit = 0U;
    uint32_t start_idx = 0U;
    
    /* Validate inputs */
    if ((str == NULL) || (result == NULL)) {
        return PARSE_ERR_NULL;
    }
    
    /* Initialize result */
    *result = 0;
    
    /* Check string length */
    len = safe_strlen(str, MAX_FIELD_LEN);
    if (len == 0U) {
        return PARSE_OK; /* Empty string = 0 */
    }
    
    /* Skip leading sign */
    if ((str[0] == '+') || (str[0] == '-')) {
        start_idx = 1U;
    }
    
    /* Validate all characters are digits */
    for (i = start_idx; i < len; i++) {
        if (is_valid_digit(str[i]) != 0U) {
            has_digit = 1U;
        } else {
            return PARSE_ERR_FORMAT;
        }
    }
    
    /* Must have at least one digit */
    if (has_digit == 0U) {
        return PARSE_ERR_FORMAT;
    }
    
    /* Manual parsing */
    {
        int32_t value = 0;
        uint8_t is_negative = 0U;
        
        if (str[0] == '-') {
            is_negative = 1U;
            i = 1U;
        } else if (str[0] == '+') {
            i = 1U;
        } else {
            i = 0U;
        }
        
        for (; i < len; i++) {
            if (is_valid_digit(str[i]) != 0U) {
                uint8_t digit = (uint8_t)(str[i] - '0');
                value = (value * 10) + (int32_t)digit;
            } else {
                return PARSE_ERR_FORMAT;
            }
        }
        
        if (is_negative != 0U) {
            value = -value;
        }
        
        *result = value;
    }
    
    return PARSE_OK;
}

/*
 * safe_string_copy: Bounded string copy with validation
 */
static parse_result_t safe_string_copy(char *dest, uint32_t dest_size, const char *src)
{
    uint32_t i = 0U;
    
    if ((dest == NULL) || (src == NULL)) {
        return PARSE_ERR_NULL;
    }
    
    if (dest_size == 0U) {
        return PARSE_ERR_BOUNDS;
    }
    
    /* Copy with bounds checking */
    for (i = 0U; i < (dest_size - 1U); i++) {
        if (src[i] == '\0') {
            break;
        }
        dest[i] = src[i];
    }
    
    /* Ensure null termination */
    dest[i] = '\0';
    
    return PARSE_OK;
}

/*
 * split_nmea_fields: Split NMEA sentence into fields
 */
static parse_result_t split_nmea_fields(char *sentence, 
                                        char *fields[MAX_FIELD_COUNT],
                                        uint32_t *field_count)
{
    uint32_t count = 0U;
    uint32_t iter = 0U;
    char *ptr = NULL;
    
    /* Validate inputs */
    if ((sentence == NULL) || (fields == NULL) || (field_count == NULL)) {
        return PARSE_ERR_NULL;
    }
    
    /* Initialize output */
    *field_count = 0U;
    for (count = 0U; count < MAX_FIELD_COUNT; count++) {
        fields[count] = NULL;
    }
    
    count = 0U;
    ptr = sentence;
    
    /* Split fields with bounded iteration */
    while ((iter < MAX_STRING_ITER) && (*ptr != '\0') && (count < MAX_FIELD_COUNT)) {
        uint32_t inner_iter = 0U;
        
        /* Mark start of field */
        fields[count] = ptr;
        count++;
        
        /* Find end of field */
        while ((inner_iter < MAX_STRING_ITER) && (*ptr != '\0') && (*ptr != ',')) {
            ptr++;
            inner_iter++;
        }
        
        /* Replace comma with null terminator */
        if (*ptr == ',') {
            *ptr = '\0';
            ptr++;
        }
        
        iter++;
    }
    
    *field_count = count;
    
    return PARSE_OK;
}

/*
 * parse_gnrmc_fields: Parse GNRMC sentence fields
 */
static parse_result_t parse_gnrmc_fields(char *fields[MAX_FIELD_COUNT],
                                         uint32_t field_count,
                                         nmea_gnrmc_t *rmc)
{
    parse_result_t result = PARSE_OK;
    int32_t speed = 0;
    int32_t course = 0;
    
    if ((fields == NULL) || (rmc == NULL)) {
        return PARSE_ERR_NULL;
    }
    
    if (field_count < 10U) {
        return PARSE_ERR_BOUNDS;
    }
    
    /* Field 1: UTC Time */
    if (fields[1] != NULL) {
        result = safe_string_copy(rmc->utc_time, sizeof(rmc->utc_time), fields[1]);
        if (result != PARSE_OK) {
            return result;
        }
    }
    
    /* Field 2: Status */
    if ((fields[2] != NULL) && (fields[2][0] != '\0')) {
        rmc->status = fields[2][0];
    } else {
        rmc->status = 'V'; /* Invalid */
    }
    
    /* Field 3: Latitude */
    if (fields[3] != NULL) {
        result = safe_string_copy(rmc->lat, sizeof(rmc->lat), fields[3]);
        if (result != PARSE_OK) {
            return result;
        }
    }
    
    /* Field 4: N/S */
    if ((fields[4] != NULL) && (fields[4][0] != '\0')) {
        rmc->ns = fields[4][0];
    } else {
        rmc->ns = 'N';
    }
    
    /* Field 5: Longitude */
    if (fields[5] != NULL) {
        result = safe_string_copy(rmc->lon, sizeof(rmc->lon), fields[5]);
        if (result != PARSE_OK) {
            return result;
        }
    }
    
    /* Field 6: E/W */
    if ((fields[6] != NULL) && (fields[6][0] != '\0')) {
        rmc->ew = fields[6][0];
    } else {
        rmc->ew = 'E';
    }
    
    /* Field 7: Speed (convert to fixed-point) */
    if (fields[7] != NULL) {
        result = parse_fixed_int(fields[7], &speed, SPEED_SCALE);
        if (result == PARSE_OK) {
            rmc->speed_knots = (uint16_t)speed;  /* Now stores speed * 100 */
        } else {
            rmc->speed_knots = 0;
        }
    } else {
        rmc->speed_knots = 0;
    }
    
    /* Field 8: Course (convert to fixed-point) */
    if (fields[8] != NULL) {
        result = parse_fixed_int(fields[8], &course, COURSE_SCALE);
        if (result == PARSE_OK) {
            rmc->course_deg = (uint16_t)course;  /* Now stores course * 100 */
        } else {
            rmc->course_deg = 0;
        }
    } else {
        rmc->course_deg = 0;
    }
    
    /* Field 9: Date */
    if (fields[9] != NULL) {
        result = safe_string_copy(rmc->date, sizeof(rmc->date), fields[9]);
        if (result != PARSE_OK) {
            return result;
        }
    }
    
    return PARSE_OK;
}

/*
 * parse_gngga_fields: Parse GNGGA sentence fields
 */
static parse_result_t parse_gngga_fields(char *fields[MAX_FIELD_COUNT],
                                         uint32_t field_count,
                                         nmea_gngga_t *gga)
{
    parse_result_t result = PARSE_OK;
    int32_t lat = 0;
    int32_t lon = 0;
    int32_t fix = 0;
    int32_t sats = 0;
    int32_t hdop = 0;
    int32_t alt = 0;
    
    if ((fields == NULL) || (gga == NULL)) {
        return PARSE_ERR_NULL;
    }
    
    if (field_count < 10U) {
        return PARSE_ERR_BOUNDS;
    }
    
    /* Field 1: UTC Time */
    if (fields[1] != NULL) {
        result = safe_string_copy(gga->utc_time, sizeof(gga->utc_time), fields[1]);
        if (result != PARSE_OK) {
            return result;
        }
    }
    
    /* Field 2: Latitude (convert to fixed-point) */
    if (fields[2] != NULL) {
        result = parse_fixed_int(fields[2], &lat, COORD_SCALE);
        gga->lat = (result == PARSE_OK) ? lat : 0;
    } else {
        gga->lat = 0;
    }
    
    /* Field 3: N/S */
    if ((fields[3] != NULL) && (fields[3][0] != '\0')) {
        gga->ns = fields[3][0];
    } else {
        gga->ns = 'N';
    }
    
    /* Field 4: Longitude (convert to fixed-point) */
    if (fields[4] != NULL) {
        result = parse_fixed_int(fields[4], &lon, COORD_SCALE);
        gga->lon = (result == PARSE_OK) ? lon : 0;
    } else {
        gga->lon = 0;
    }
    
    /* Field 5: E/W */
    if ((fields[5] != NULL) && (fields[5][0] != '\0')) {
        gga->ew = fields[5][0];
    } else {
        gga->ew = 'E';
    }
    
    /* Field 6: Fix Quality */
    if (fields[6] != NULL) {
        result = parse_int(fields[6], &fix);
        gga->fix_quality = (result == PARSE_OK) ? (int)fix : 0;
    } else {
        gga->fix_quality = 0;
    }
    
    /* Field 7: Number of Satellites */
    if (fields[7] != NULL) {
        result = parse_int(fields[7], &sats);
        gga->num_satellites = (result == PARSE_OK) ? (int)sats : 0;
    } else {
        gga->num_satellites = 0;
    }
    
    /* Field 8: HDOP (convert to fixed-point) */
    if (fields[8] != NULL) {
        result = parse_fixed_int(fields[8], &hdop, HDOP_SCALE);
        gga->hdop = (result == PARSE_OK) ? hdop : 0;
    } else {
        gga->hdop = 0;
    }
    
    /* Field 9: Altitude (convert to fixed-point) */
    if (fields[9] != NULL) {
        result = parse_fixed_int(fields[9], &alt, ALT_SCALE);
        gga->altitude = (result == PARSE_OK) ? alt : 0;
    } else {
        gga->altitude = 0;
    }
    
    return PARSE_OK;
}

/*
 * parse_gnvtg_fields: Parse GNVTG sentence fields
 */
static parse_result_t parse_gnvtg_fields(char *fields[MAX_FIELD_COUNT],
                                         uint32_t field_count,
                                         nmea_gnvtg_t *vtg)
{
    parse_result_t result = PARSE_OK;
    int32_t course_t = 0;
    int32_t course_m = 0;
    int32_t speed_kn = 0;
    int32_t speed_km = 0;
    
    if ((fields == NULL) || (vtg == NULL)) {
        return PARSE_ERR_NULL;
    }
    
    if (field_count < 9U) {
        return PARSE_ERR_BOUNDS;
    }
    
    /* Field 1: Course True */
    if (fields[1] != NULL) {
        result = parse_fixed_int(fields[1], &course_t, COURSE_SCALE);
        vtg->course_true = (result == PARSE_OK) ? course_t : 0;
    } else {
        vtg->course_true = 0;
    }
    
    /* Field 3: Course Magnetic */
    if (fields[3] != NULL) {
        result = parse_fixed_int(fields[3], &course_m, COURSE_SCALE);
        vtg->course_magnetic = (result == PARSE_OK) ? course_m : 0;
    } else {
        vtg->course_magnetic = 0;
    }
    
    /* Field 5: Speed Knots */
    if (fields[5] != NULL) {
        result = parse_fixed_int(fields[5], &speed_kn, SPEED_SCALE);
        vtg->speed_knots = (result == PARSE_OK) ? speed_kn : 0;
    } else {
        vtg->speed_knots = 0;
    }
    
    /* Field 7: Speed km/h */
    if (fields[7] != NULL) {
        result = parse_fixed_int(fields[7], &speed_km, SPEED_SCALE);
        vtg->speed_kmh = (result == PARSE_OK) ? speed_km : 0;
    } else {
        vtg->speed_kmh = 0;
    }
    
    return PARSE_OK;
}

/*
 * nmea_parse_sentence: Main parsing function
 * Integer-only version for RP2040
 */
nmea_type_t nmea_parse_sentence(char *sentence,
                                nmea_gnrmc_t *rmc,
                                nmea_gngga_t *gga,
                                nmea_gnvtg_t *vtg)
{
    char buffer[MAX_SENTENCE_LEN];
    char *fields[MAX_FIELD_COUNT];
    uint32_t field_count = 0U;
    nmea_type_t sentence_type = NMEA_TYPE_UNKNOWN;
    parse_result_t result = PARSE_OK;
    uint32_t i = 0U;
    
    /* Validate input */
    if (sentence == NULL) {
        return NMEA_TYPE_UNKNOWN;
    }
    
    /* Determine sentence type */
    if (strstr(sentence, "$GNRMC") != NULL) {
        sentence_type = NMEA_TYPE_GNRMC;
        if (rmc == NULL) {
            return NMEA_TYPE_UNKNOWN;
        }
    } else if (strstr(sentence, "$GNGGA") != NULL) {
        sentence_type = NMEA_TYPE_GNGGA;
        if (gga == NULL) {
            return NMEA_TYPE_UNKNOWN;
        }
    } else if (strstr(sentence, "$GNVTG") != NULL) {
        sentence_type = NMEA_TYPE_GNVTG;
        if (vtg == NULL) {
            return NMEA_TYPE_UNKNOWN;
        }
    } else {
        return NMEA_TYPE_UNKNOWN;
    }
    
    /* Initialize buffer */
    for (i = 0U; i < MAX_SENTENCE_LEN; i++) {
        buffer[i] = '\0';
    }
    
    /* Copy to buffer */
    result = safe_string_copy(buffer, sizeof(buffer), sentence);
    if (result != PARSE_OK) {
        return NMEA_TYPE_UNKNOWN;
    }
    
    /* Split into fields */
    result = split_nmea_fields(buffer, fields, &field_count);
    if (result != PARSE_OK) {
        return NMEA_TYPE_UNKNOWN;
    }
    
    /* Parse based on type */
    if (sentence_type == NMEA_TYPE_GNRMC) {
        result = parse_gnrmc_fields(fields, field_count, rmc);
    } else if (sentence_type == NMEA_TYPE_GNGGA) {
        result = parse_gngga_fields(fields, field_count, gga);
    } else if (sentence_type == NMEA_TYPE_GNVTG) {
        result = parse_gnvtg_fields(fields, field_count, vtg);
    } else {
        return NMEA_TYPE_UNKNOWN;
    }
    
    /* Check parse result */
    if (result != PARSE_OK) {
        return NMEA_TYPE_UNKNOWN;
    }
    
    return sentence_type;
}
