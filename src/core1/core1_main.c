#include "core1_main.h"

#define TX_POOL_SIZE 8

static TLM_serial_rcv_t tx_pool[TX_POOL_SIZE];
static volatile uint8_t tx_index = 0;

void queue_tlm_to_core(TLM_serial_rcv_t *src) 
{
    uint8_t i = tx_index++ % TX_POOL_SIZE;
    memcpy(&tx_pool[i], src, sizeof(TLM_serial_rcv_t));
    multicore_fifo_push_blocking((uint32_t)&tx_pool[i]);
}

void receive_tlm_from_core0(TLM_serial_rcv_t *dest) 
{
    TLM_serial_rcv_t *src = (TLM_serial_rcv_t *)multicore_fifo_pop_blocking();
    memcpy(dest, src, sizeof(TLM_serial_rcv_t));
}


void core1_entry(void) {
    TLM_serial_rcv_t received_packet;
    uint32_t rcvd_packet_count = 0;
    
    while (true) 
    {
        // Receive packet from Core 0
        receive_tlm_from_core0(&received_packet);
        rcvd_packet_count++;
        
        // Verify CRC
        uint16_t calculated_crc = 0xFFFF;
        uint8_t* data = (uint8_t*)&received_packet.packet;
        for (size_t i = 0; i < sizeof(TLM_packet_rcv_t); i++) {
            calculated_crc ^= ((uint16_t)data[i] << 8);
            for (uint8_t j = 0; j < 8; j++) {
                if (calculated_crc & 0x8000) 
                    calculated_crc = (calculated_crc << 1) ^ 0x1021;
                else 
                    calculated_crc <<= 1;
            }
        }
        
        bool crc_valid = (calculated_crc == received_packet.CRC16);
        bool header_valid = (received_packet.header == FIFO_HEADER_UINT16);

        // Process packet here (SD card write, etc.)
        // For now, just echo back to Core 0
        queue_tlm_to_core(&received_packet);
    }
}
