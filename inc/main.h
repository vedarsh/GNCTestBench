#ifndef __MAIN_H__
#define __MAIN_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "hardware/divider.h"

#include "QMC5883L/qmc5883l.h"
#include "NEOM8N/neom8n.h"
#include "ICM45686/ICM45686.h"

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

#endif