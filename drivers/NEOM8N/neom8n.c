#include "neom8n.h"

//set up dma to receive from uart
void uart_dma_init(uart_inst_t *uart, uint8_t *rx_buf, size_t buf_size, int *dma_chan) {

    // Set up DMA for UART RX
    *dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(*dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(uart, false)); // RX DREQ 

    dma_channel_configure(
        *dma_chan,
        &c,
        rx_buf,               // Destination pointer
        &uart_get_hw(uart)->dr, // Source pointer
        buf_size,            // Number of bytes to transfer
        true                 // Start immediately
    );
}

void start_uart_dma(uart_inst_t *uart, int dma_chan) {
    // Enable UART RX interrupt
    uart_set_irq_enables(uart, true, false);
    irq_set_exclusive_handler(UART0_IRQ, NULL); // No handler, just for DMA
    irq_set_enabled(UART0_IRQ, true);
}   
