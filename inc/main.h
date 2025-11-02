#ifndef __MAIN_H__
#define __MAIN_H_

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/critical_section.h"
#include "hardware/watchdog.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "PacketStructures.h"

/* External sensor drivers */
#include "ICM45686/icm45686.h"      /* IMU driver */
#include "QMC5883L/qmc5883l.h"      /* Magnetometer driver */
#include "NEOM8N/neom8n.h"          /* GPS NMEA parser */


/*============================================================================*/
/* PIN CONFIG PARAMETERS                                                      */
/*============================================================================*/

#define I2C_PORT i2c0
#define I2C_SDA 1
#define I2C_SCL 0
#define I2C_TIMEOUT_US 50000U

#define NEOM8N_UART_TX_PIN 12
#define NEOM8N_UART_RX_PIN 13
#define NEOM8N_UART_BAUDRATE 9600U


/*============================================================================*/
/* GPS TIME SYNCHRONIZATION                                                   */
/*============================================================================*/

/**
 * @brief Time synchronization state structure
 */
typedef struct {
    bool synced;                    /**< Time synchronization status */
    uint32_t gps_epoch_ms;          /**< GPS time in milliseconds since GPS epoch */
    uint64_t system_boot_offset_ms; /**< System boot time offset */
    absolute_time_t last_sync_time; /**< Last successful GPS time sync */
} time_sync_state_t;

#endif