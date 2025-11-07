#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* DATA STRUCTURES                                                            */
/*============================================================================*/

#pragma pack(push,1)
typedef struct {
    qmc_5883_mag_read_t mag;
    nmea_gnrmc_t gnrmc;
    nmea_gngga_t gngga;
    nmea_gnvtg_t gnvtg;
} TLM_packet_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    uint16_t header;
    TLM_packet_t packet;
    uint16_t CRC16;
} TLM_serial_t;
#pragma pack(pop)

typedef enum {
    DEV_GPS = 0,
    DEV_MAG,
    DEV_RTC,
    DEV_IMU,
    DEV_COUNT
} current_device_t;