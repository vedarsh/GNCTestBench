#ifndef __CONFIG_H__
#define __CONFIG_H__

/*============================================================================*/
/* CONFIGURATION PARAMETERS                                                   */
/*============================================================================*/

#define I2C_PORT i2c0
#define I2C_SDA 5
#define I2C_SCL 4

#define NEOM8N_UART_TX_PIN 12
#define NEOM8N_UART_RX_PIN 13
#define NEOM8N_UART_BAUDRATE 9600

#define NMEA_BUFFER_SIZE 128
#define NMEA_NUM_PREFIXES 3
#define MAX_PREFIX_LEN 6

#define GPS_SYNC_TIMEOUT_MS 120000

#define TELEMETRY_PRINT_INTERVAL 1000

#define WATCHDOG_TIMEOUT_MS 8000

#define FIFO_HEADER_UINT16 0xABAB

#define TX_POOL_SIZE 8

#endif
