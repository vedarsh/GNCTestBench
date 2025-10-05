#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"


void neom8n_init(uart_inst_t *uart, uint8_t baudrate, uint8_t *rx_buf, size_t buf_size, int *dma_chan) ;