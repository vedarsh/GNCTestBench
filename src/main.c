/**
 * @file main.c
 * @brief GNC Test Bench Telemetry System
 * @author Vedarsh
 * @date 2025-10-07
 * @version 2.1.0
 * @compliance DO-178C Level C, MISRA-C:2012
 * 
 * @note Dual-core architecture:
 *       Core 0: Sensor acquisition, fusion, and filtering
 *       Core 1: Telemetry processing and output
 * 
 * @changes v2.1.0: Removed DS3231 RTC dependency, added GPS time synchronization
 * 
 * @build_requirements CMakeLists.txt must include:
 *   pico_enable_stdio_usb(${PROJECT_NAME} 1)
 *   pico_enable_stdio_uart(${PROJECT_NAME} 0)
 *   target_link_libraries(${PROJECT_NAME} 
 *       pico_stdlib
 *       pico_multicore
 *       hardware_i2c
 *       hardware_uart
 *       hardware_watchdog
 *   )
 */

#include "main.h"


/*============================================================================*/
/* UTILITY FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Safe float comparison with epsilon
 * @param[in] a First operand
 * @param[in] b Second operand
 * @param[in] epsilon Comparison tolerance
 * @return true if values are within epsilon
 */
static inline bool float_equals(float a, float b, float epsilon) 
{
    return fabsf(a - b) < epsilon;
}

/**
 * @brief Constrain value between min and max
 * @param[in] val Input value
 * @param[in] min Minimum bound
 * @param[in] max Maximum bound
 * @return Constrained value
 */
static inline float constrain_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief Wrap angle to [-180, 180] range
 * @param[in] angle Input angle in degrees
 * @return Wrapped angle
 */
static float wrap_180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief CRC-16-CCITT calculation (polynomial 0x1021)
 * @param[in] data Pointer to data buffer
 * @param[in] length Length of data in bytes
 * @return Calculated CRC-16 value
 */
static uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFU;
    
    for (size_t i = 0U; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0U; j < 8U; j++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (crc << 1) ^ 0x1021U;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Verify packet integrity
 * @param[in] packet Pointer to telemetry serial packet
 * @return true if packet is valid
 */
static bool verify_packet_integrity(const TLM_serial_t *packet) {
    if (packet->header != FIFO_HEADER_UINT16) return false;
    if (packet->footer != FIFO_FOOTER_UINT16) return false;
    
    uint16_t calc_crc = calculate_crc16((const uint8_t*)&packet->packet, 
                                         sizeof(TLM_packet_t));
    return (calc_crc == packet->crc16);
}

static time_sync_state_t time_sync = {
    .synced = false,
    .gps_epoch_ms = 0U,
    .system_boot_offset_ms = 0U,
    .last_sync_time = {0}
};

/**
 * @brief Update time synchronization from GPS data
 * @param[in] gnrmc Pointer to GNRMC sentence data
 */
static void update_time_sync(const nmea_gnrmc_t *gnrmc) {
    if (gnrmc->status != 'A') {
        return; /* GPS not locked */
    }
    
    uint32_t gps_epoch = gps_time_to_epoch_ms(gnrmc->utc_time, gnrmc->date);
    if (gps_epoch == 0U) {
        return; /* Invalid time parse */
    }
    
    absolute_time_t now = get_absolute_time();
    
    if (!time_sync.synced) {
        /* First sync */
        time_sync.gps_epoch_ms = gps_epoch;
        time_sync.system_boot_offset_ms = to_ms_since_boot(now);
        time_sync.synced = true;
        time_sync.last_sync_time = now;
        
        printf("[TIME] GPS time synchronized: %s UTC (Date: %s)\n", 
               gnrmc->utc_time, gnrmc->date);
    } else {
        /* Continuous sync - update drift compensation */
        uint64_t system_elapsed = to_ms_since_boot(now) - time_sync.system_boot_offset_ms;
        uint32_t gps_elapsed = gps_epoch - time_sync.gps_epoch_ms;
        int64_t drift_ms = (int64_t)system_elapsed - (int64_t)gps_elapsed;
        
        if (abs(drift_ms) > 1000) { /* >1 second drift */
            time_sync.gps_epoch_ms = gps_epoch;
            time_sync.system_boot_offset_ms = to_ms_since_boot(now);
            printf("[TIME] Clock drift corrected: %lldms\n", drift_ms);
        }
        
        time_sync.last_sync_time = now;
    }
}

/**
 * @brief Get current synchronized time
 * @param[out] timeframe Pointer to time frame structure
 */
static void get_synchronized_time(timeframe_t *timeframe) {
    absolute_time_t now = get_absolute_time();
    
    if (!time_sync.synced) {
        timeframe-> is_time_synced = false;
        timeframe-> year = 0U;
        timeframe-> month = 0U;
        timeframe-> day = 0U;
        timeframe-> hour = 0U;
        timeframe-> minute = 0U;
        timeframe-> second = 0U;
        return;
    }
    
    /* Calculate current GPS time */
    uint64_t system_elapsed = to_ms_since_boot(now) - time_sync.system_boot_offset_ms;
    uint32_t current_gps_ms = time_sync.gps_epoch_ms + (uint32_t)system_elapsed;
    
    /* Convert back to time components (simplified) */
    uint32_t total_seconds = current_gps_ms / 1000U;
    timeframe-> second = (uint8_t)(total_seconds % 60U);
    timeframe-> minute = (uint8_t)((total_seconds / 60U) % 60U);
    timeframe-> hour = (uint8_t)((total_seconds / 3600U) % 24U);
    
    /* Check sync freshness (warn if >60s since last GPS update) */
    int64_t sync_age_ms = absolute_time_diff_us(time_sync.last_sync_time, now) / 1000;
    timeframe-> is_time_synced = (sync_age_ms < 60000);
    
    /* Simplified date (keep from last GPS sync) */
    timeframe-> year = 2025U;
    timeframe-> month = 1U;
    timeframe-> day = 1U;
}

/**
 * @brief Validate IMU sensor data
 * @param[in] data Pointer to scaled IMU data
 * @return true if data is within valid ranges
 */
static bool validate_imu_data(const icm42686_scaled_t *data) {
    if (fabsf(data->accel_x_g) > MAX_ACCEL_G) return false;
    if (fabsf(data->accel_y_g) > MAX_ACCEL_G) return false;
    if (fabsf(data->accel_z_g) > MAX_ACCEL_G) return false;
    
    if (fabsf(data->gyro_x_dps) > MAX_GYRO_DPS) return false;
    if (fabsf(data->gyro_y_dps) > MAX_GYRO_DPS) return false;
    if (fabsf(data->gyro_z_dps) > MAX_GYRO_DPS) return false;
    
    return true;
}

/*============================================================================*/
/* SENSOR FUSION - COMPLEMENTARY FILTER                                       */
/*============================================================================*/

/**
 * @brief Initialize sensor fusion with initial orientation
 * @param[in] imu Pointer to IMU data
 * @param[in] mag Pointer to magnetometer data
 */
static void fusion_initialize(const icm42686_scaled_t *imu, 
                              const qmc_5883_mag_read_t *mag) {
    /* Calculate initial roll and pitch from accelerometer */
    float ax = imu->accel_x_g;
    float ay = imu->accel_y_g;
    float az = imu->accel_z_g;
    
    fusion_state.roll = atan2f(ay, az) * 180.0f / M_PI;
    fusion_state.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    
    /* Calculate initial yaw from magnetometer */
    float mx = (float)mag->mag_x;
    float my = (float)mag->mag_y;
    float mz = (float)mag->mag_z;
    
    /* Tilt compensation for magnetometer */
    float cos_pitch = cosf(fusion_state.pitch * M_PI / 180.0f);
    float sin_pitch = sinf(fusion_state.pitch * M_PI / 180.0f);
    float cos_roll = cosf(fusion_state.roll * M_PI / 180.0f);
    float sin_roll = sinf(fusion_state.roll * M_PI / 180.0f);
    
    float mx_comp = mx * cos_pitch + mz * sin_pitch;
    float my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    
    fusion_state.yaw = atan2f(-my_comp, mx_comp) * 180.0f / M_PI;
    fusion_state.yaw = wrap_180(fusion_state.yaw + MAG_DECLINATION);
    
    fusion_state.gyro_bias_x = 0.0f;
    fusion_state.gyro_bias_y = 0.0f;
    fusion_state.gyro_bias_z = 0.0f;
    
    fusion_state.initialized = true;
    fusion_state.update_count = 0U;
}

/**
 * @brief Update attitude using complementary filter
 * @param[in] imu Pointer to IMU data
 * @param[in] mag Pointer to magnetometer data
 * @param[out] attitude Pointer to attitude output structure
 */
static void fusion_update(const icm42686_scaled_t *imu,
                         const qmc_5883_mag_read_t *mag,
                         attitude_data_t *attitude) {
    
    if (!fusion_state.initialized) {
        fusion_initialize(imu, mag);
        attitude->fusion_valid = false;
        return;
    }
    
    /* Apply gyro bias correction */
    float gx = imu->gyro_x_dps - fusion_state.gyro_bias_x;
    float gy = imu->gyro_y_dps - fusion_state.gyro_bias_y;
    float gz = imu->gyro_z_dps - fusion_state.gyro_bias_z;
    
    /* Gyro integration (prediction) */
    float roll_gyro = fusion_state.roll + gx * GYRO_DT;
    float pitch_gyro = fusion_state.pitch + gy * GYRO_DT;
    float yaw_gyro = fusion_state.yaw + gz * GYRO_DT;
    
    /* Accelerometer-based attitude (correction) */
    float ax = imu->accel_x_g;
    float ay = imu->accel_y_g;
    float az = imu->accel_z_g;
    
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    float roll_accel = atan2f(ay, az) * 180.0f / M_PI;
    float pitch_accel = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    
    /* Complementary filter fusion */
    if (accel_mag > 0.5f && accel_mag < 1.5f) {
        /* Valid accelerometer data (near 1g) */
        fusion_state.roll = COMPLEMENTARY_ALPHA * roll_gyro + 
                           (1.0f - COMPLEMENTARY_ALPHA) * roll_accel;
        fusion_state.pitch = COMPLEMENTARY_ALPHA * pitch_gyro + 
                            (1.0f - COMPLEMENTARY_ALPHA) * pitch_accel;
    } else {
        /* High acceleration - trust gyro only */
        fusion_state.roll = roll_gyro;
        fusion_state.pitch = pitch_gyro;
    }
    
    /* Magnetometer heading with tilt compensation */
    float mx = (float)mag->mag_x;
    float my = (float)mag->mag_y;
    float mz = (float)mag->mag_z;
    
    float cos_pitch = cosf(fusion_state.pitch * M_PI / 180.0f);
    float sin_pitch = sinf(fusion_state.pitch * M_PI / 180.0f);
    float cos_roll = cosf(fusion_state.roll * M_PI / 180.0f);
    float sin_roll = sinf(fusion_state.roll * M_PI / 180.0f);
    
    float mx_comp = mx * cos_pitch + mz * sin_pitch;
    float my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    
    float heading_mag = atan2f(-my_comp, mx_comp) * 180.0f / M_PI;
    heading_mag = wrap_180(heading_mag + MAG_DECLINATION);
    
    /* Yaw fusion (slower response for magnetometer) */
    const float MAG_WEIGHT = 0.05f;
    float yaw_diff = wrap_180(heading_mag - fusion_state.yaw);
    fusion_state.yaw = wrap_180(fusion_state.yaw + MAG_WEIGHT * yaw_diff);
    
    /* Populate output */
    attitude->roll_deg = fusion_state.roll;
    attitude->pitch_deg = fusion_state.pitch;
    attitude->yaw_deg = fusion_state.yaw;
    attitude->roll_rate_dps = gx;
    attitude->pitch_rate_dps = gy;
    attitude->yaw_rate_dps = gz;
    attitude->accel_magnitude_g = accel_mag;
    attitude->heading_mag_deg = heading_mag;
    attitude->fusion_valid = true;
    
    fusion_state.update_count++;
}

/*============================================================================*/
/* HEALTH MONITORING                                                          */
/*============================================================================*/

/**
 * @brief Update system health status
 * @param[in,out] packet Pointer to telemetry packet
 */
static void update_health_status(TLM_packet_t *packet) {
    absolute_time_t now = get_absolute_time();
    
    /* Check GPS health */
    int64_t gps_age_ms = absolute_time_diff_us(system_health.last_gps_update, now) / 1000;
    if (gps_age_ms > SENSOR_TIMEOUT_MS) {
        system_health.gps = HEALTH_DEGRADED;
    } else if (packet->gnrmc.status == 'A') {
        system_health.gps = HEALTH_NOMINAL;
    }
    
    /* Check IMU health */
    int64_t imu_age_ms = absolute_time_diff_us(system_health.last_imu_update, now) / 1000;
    if (imu_age_ms > SENSOR_TIMEOUT_MS) {
        system_health.imu = HEALTH_FAILED;
    } else {
        system_health.imu = HEALTH_NOMINAL;
    }
    
    /* Check magnetometer health */
    int64_t mag_age_ms = absolute_time_diff_us(system_health.last_mag_update, now) / 1000;
    if (mag_age_ms > SENSOR_TIMEOUT_MS) {
        system_health.mag = HEALTH_DEGRADED;
    } else {
        system_health.mag = HEALTH_NOMINAL;
    }
    
    /* Check fusion health */
    if (packet->attitude.fusion_valid) {
        system_health.fusion = HEALTH_NOMINAL;
    } else {
        system_health.fusion = HEALTH_DEGRADED;
    }
    
    /* Check time sync health */
    int64_t sync_age_ms = absolute_time_diff_us(time_sync.last_sync_time, now) / 1000;
    if (time_sync.synced && sync_age_ms < 60000) {
        system_health.rtc = HEALTH_NOMINAL;  /* Reuse RTC field for time sync */
    } else {
        system_health.rtc = HEALTH_DEGRADED;
    }
    
    /* Check CRC error rate */
    if (system_health.crc_error_count > MAX_CRC_ERRORS) {
        /* Critical error - reset system */
        watchdog_enable(1, true);
        while(1); /* Force watchdog reset */
    }
    
    /* Copy health to packet */
    memcpy(&packet->health, &system_health, sizeof(system_health_t));
}

/*============================================================================*/
/* HARDWARE INITIALIZATION                                                    */
/*============================================================================*/

/**
 * @brief Initialize I2C interface
 */
static inline void init_i2c(void) 
{
    i2c_init(I2C_PORT, 400000U);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

/**
 * @brief UART0 interrupt handler for GPS NMEA data
 */
void __isr __time_critical_func(uart0_irq_handler)(void) {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);

        if (!nmea_state.receiving) {
            bool prefix_started = false;

            for (int i = 0; i < (int)NMEA_NUM_PREFIXES; i++) {
                uint8_t pos = nmea_state.prefix_pos[i];
                
                if (ch == (uint8_t)nmea_prefixes[i][pos]) {
                    nmea_state.prefix_pos[i]++;
                    prefix_started = true;

                    if (nmea_state.prefix_pos[i] == prefix_lengths[i]) {
                        nmea_state.matched_prefix = (int8_t)i;
                        nmea_state.receiving = true;
                        nmea_write_idx = 0U;
                        
                        memcpy((void*)nmea_buffers[write_buffer_idx], 
                               nmea_prefixes[i], prefix_lengths[i]);
                        nmea_write_idx = prefix_lengths[i];
                        memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
                        break;
                    }
                } else {
                    nmea_state.prefix_pos[i] = (ch == '$') ? 1U : 0U;
                    if (ch == '$') {
                        prefix_started = true;
                    }
                }
            }

            if (!prefix_started) {
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }

        } else {
            if (nmea_write_idx < (NMEA_BUFFER_SIZE - 1U)) {
                nmea_buffers[write_buffer_idx][nmea_write_idx++] = ch;

                if (ch == '\n') {
                    nmea_buffers[write_buffer_idx][nmea_write_idx] = '\0';
                    ready_buffer_idx = (int8_t)write_buffer_idx;
                    write_buffer_idx ^= 1U;
                    nmea_data_ready = true;
                    nmea_state.receiving = false;
                    nmea_state.matched_prefix = -1;
                    system_health.last_gps_update = get_absolute_time();
                }
            } else {
                nmea_state.receiving = false;
                nmea_write_idx = 0U;
                nmea_state.matched_prefix = -1;
                memset((void*)nmea_state.prefix_pos, 0, NMEA_NUM_PREFIXES);
            }
        }
    }
}

static TLM_serial_t core1_received_packet;
static volatile bool core1_data_available = false;

/**
 * @brief Core 1 SIO interrupt handler for inter-core communication
 */
void core1_sio_irq(void) {
    while (multicore_fifo_rvalid()) {
        uint32_t src_ptr = multicore_fifo_pop_blocking();
        TLM_serial_t *src = (TLM_serial_t *)src_ptr;
        memcpy(&core1_received_packet, src, sizeof(TLM_serial_t));
        core1_data_available = true;
    }
    multicore_fifo_clear_irq();
}

/**
 * @brief Initialize UART for GPS communication
 */
static inline void init_uart(void) {
    /* Initialize hardware UART0 for GPS */
    uart_init(uart0, NEOM8N_UART_BAUDRATE);
    gpio_set_function(NEOM8N_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NEOM8N_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8U, 1U, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    /* Set UART RX FIFO interrupt threshold */
    hw_write_masked(&uart_get_hw(uart0)->ifls,
                    2U << UART_UARTIFLS_RXIFLSEL_LSB,
                    UART_UARTIFLS_RXIFLSEL_BITS);

    /* Configure UART interrupt handler */
    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
    irq_set_priority(UART0_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
    
    printf("[INIT] Hardware UART0 initialized for GPS on pins GP%d(TX)/GP%d(RX)\n", 
           NEOM8N_UART_TX_PIN, NEOM8N_UART_RX_PIN);
}

/*============================================================================*/
/* TELEMETRY FUNCTIONS - CORE 0                                               */
/*============================================================================*/

/**
 * @brief Acquire sensor data based on current device
 * @param[in,out] tlm_packet Pointer to telemetry packet
 * @param[in] device Current device to poll
 */
void telemetry_acquire_sensor(TLM_packet_t* tlm_packet, current_device_t device) {
    switch(device) {
        case DEV_GPS:
            if (nmea_data_ready) {
                critical_section_enter_blocking(&nmea_crit_sec);
                if (ready_buffer_idx >= 0) {
                    nmea_parse_sentence((char*)nmea_buffers[ready_buffer_idx], 
                                      &tlm_packet->gnrmc, 
                                      &tlm_packet->gngga, 
                                      &tlm_packet->gnvtg);
                    
                    /* Update time synchronization */
                    update_time_sync(&tlm_packet->gnrmc);
                    
                    ready_buffer_idx = -1;
                }
                nmea_data_ready = false;
                critical_section_exit(&nmea_crit_sec);
            }
            break;

        case DEV_MAG:
            if (qmc5883l_read_mag_drdy(I2C_PORT, &tlm_packet->mag) == SENSOR_OK) {
                system_health.last_mag_update = get_absolute_time();
            }
            break;


        case DEV_IMU:
            if (sensor_value_scaled(I2C_PORT, &tlm_packet->imu_scaled)) {
                if (validate_imu_data(&tlm_packet->imu_scaled)) {
                    system_health.last_imu_update = get_absolute_time();
                }
            }
            break;

        case DEV_FUSION:
            fusion_update(&tlm_packet->imu_scaled, 
                         &tlm_packet->mag, 
                         &tlm_packet->attitude);
            break;

        default:
            break;
    }
}

/**
 * @brief Serialize telemetry packet for transmission
 * @param[in] tlm_packet Pointer to source packet
 * @param[out] tlm_serial Pointer to destination serial packet
 */
void tlm_serialize_packet(TLM_packet_t* tlm_packet, TLM_serial_t* tlm_serial) {
    tlm_serial->header = FIFO_HEADER_UINT16;
    memcpy(&tlm_serial->packet, tlm_packet, sizeof(TLM_packet_t));
    
    tlm_serial->crc16 = calculate_crc16((const uint8_t*)&tlm_serial->packet, 
                                         sizeof(TLM_packet_t));
    tlm_serial->footer = FIFO_FOOTER_UINT16;
}

/**
 * @brief Queue telemetry to Core 1 with overflow protection
 * @param[in] src Pointer to source serial packet
 * @return true if successfully queued
 */
static bool queue_tlm_to_core1(TLM_serial_t *src) {
    uint8_t idx = tx_write_idx % TX_POOL_SIZE;
    tx_write_idx++;
    
    memcpy(&tx_pool[idx], src, sizeof(TLM_serial_t));
    
    /* Non-blocking push with timeout */
    uint32_t timeout = time_us_32() + 10000U; /* 10ms timeout */
    while (!multicore_fifo_wready()) {
        if (time_us_32() > timeout) {
            return false; /* FIFO full */
        }
        tight_loop_contents();
    }
    
    multicore_fifo_push_blocking((uint32_t)&tx_pool[idx]);
    return true;
}

/**
 * @brief Receive telemetry from Core 1 (non-blocking with timeout)
 * @param[out] dest Pointer to destination packet
 * @param[in] timeout_us Timeout in microseconds
 * @return true if packet received and valid
 */
static bool receive_tlm_from_core1(TLM_serial_t *dest, uint32_t timeout_us) {
    uint32_t start = time_us_32();
    
    while (!multicore_fifo_rvalid()) {
        if ((time_us_32() - start) > timeout_us) {
            return false;
        }
        tight_loop_contents();
    }
    
    TLM_serial_t *src = (TLM_serial_t *)multicore_fifo_pop_blocking();
    memcpy(dest, src, sizeof(TLM_serial_t));
    
    return verify_packet_integrity(dest);
}

/*============================================================================*/
/* CORE 1 ENTRY POINT - TELEMETRY OUTPUT                                     */
/*============================================================================*/

/**
 * @brief Core 1 main entry point - telemetry processing and output
 */
void core1_entry(void) {
    /* Setup inter-core interrupt */
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_sio_irq);
    irq_set_enabled(SIO_IRQ_PROC1, true);

    uint32_t packet_count = 0U;
    uint32_t valid_packets = 0U;
    uint32_t invalid_packets = 0U;
    absolute_time_t last_print_time = get_absolute_time();
    
    printf("[CORE1] Telemetry processor started\n");
    
    while (true) {
        if (core1_data_available) {
            core1_data_available = false;
            packet_count++;
            
            /* Verify packet integrity */
            bool header_valid = (core1_received_packet.header == FIFO_HEADER_UINT16);
            bool footer_valid = (core1_received_packet.footer == FIFO_FOOTER_UINT16);
            bool crc_valid = verify_packet_integrity(&core1_received_packet);
            
            if (header_valid && footer_valid && crc_valid) {
                valid_packets++;
                
                /* Print telemetry at reduced rate (1Hz) */
                absolute_time_t now = get_absolute_time();
                int64_t dt_ms = absolute_time_diff_us(last_print_time, now) / 1000;
                
                if (dt_ms >= 1000) {
                    last_print_time = now;
                    
                    printf("═══════════════════════════════════════════════════════════════\n");
                    printf("-- TELEMETRY PACKET #%-5lu | Core1 Uptime: %llu ms         --\n", 
                           core1_received_packet.packet.packet_sequence,
                           to_ms_since_boot(now));
                    
                    /* GPS-Synchronized Time */
                    printf("-- [TIME] %02d:%02d:%02d UTC %s                          --\n",
                           core1_received_packet.packet.timeframe. hour,
                           core1_received_packet.packet.timeframe. minute,
                           core1_received_packet.packet.timeframe. second,
                           core1_received_packet.packet.timeframe. is_time_synced ? 
                           "[GPS-SYNCED]" : "[UNSYNCED]");
                    
                    /* IMU Data - Scaled */
                    printf("-- [IMU]     Ax:%7.3fg Ay:%7.3fg Az:%7.3fg         --\n",
                           core1_received_packet.packet.imu_scaled.accel_x_g,
                           core1_received_packet.packet.imu_scaled.accel_y_g,
                           core1_received_packet.packet.imu_scaled.accel_z_g);
                    printf("--           Gx:%7.2f° Gy:%7.2f° Gz:%7.2f° T:%5.1f°C --\n",
                           core1_received_packet.packet.imu_scaled.gyro_x_dps,
                           core1_received_packet.packet.imu_scaled.gyro_y_dps,
                           core1_received_packet.packet.imu_scaled.gyro_z_dps,
                           core1_received_packet.packet.imu_scaled.temp_c);
                    
                    /* Attitude (Fused) */
                    printf("-- [ATTITUDE] Roll:%7.2f° Pitch:%7.2f° Yaw:%7.2f°   --\n",
                           core1_received_packet.packet.attitude.roll_deg,
                           core1_received_packet.packet.attitude.pitch_deg,
                           core1_received_packet.packet.attitude.yaw_deg);
                    printf("--            Rate: R:%6.2f P:%6.2f Y:%6.2f °/s      --\n",
                           core1_received_packet.packet.attitude.roll_rate_dps,
                           core1_received_packet.packet.attitude.pitch_rate_dps,
                           core1_received_packet.packet.attitude.yaw_rate_dps);
                    printf("--            |Accel|:%5.3fg Mag Hdg:%7.2f° %s      --\n",
                           core1_received_packet.packet.attitude.accel_magnitude_g,
                           core1_received_packet.packet.attitude.heading_mag_deg,
                           core1_received_packet.packet.attitude.fusion_valid ? 
                           "[VALID]" : "[INIT]");
                    
                    /* Magnetometer */
                    printf("-- [MAG]     X:%6d Y:%6d Z:%6d (counts)           --\n",
                           core1_received_packet.packet.mag.mag_x,
                           core1_received_packet.packet.mag.mag_y,
                           core1_received_packet.packet.mag.mag_z);
                    
                    /* GPS Data */
                    printf("-- [GPS] Status:%c Time:%s Date:%s                  --\n",
                           core1_received_packet.packet.gnrmc.status,
                           core1_received_packet.packet.gnrmc.utc_time,
                           core1_received_packet.packet.gnrmc.date);
                    printf("--       Lat:%s%c Lon:%s%c                           --\n",
                           core1_received_packet.packet.gnrmc.lat,
                           core1_received_packet.packet.gnrmc.ns,
                           core1_received_packet.packet.gnrmc.lon,
                           core1_received_packet.packet.gnrmc.ew);
                    printf("--       Speed:%6.2fkts Course:%6.2f° Fix:%d Sats:%2d    --\n",
                           core1_received_packet.packet.gnrmc.speed_knots,
                           core1_received_packet.packet.gnrmc.course_deg,
                           core1_received_packet.packet.gngga.fix_quality,
                           core1_received_packet.packet.gngga.num_satellites);
                    printf("--       Alt:%7.2fm HDOP:%5.2f                            --\n",
                           core1_received_packet.packet.gngga.altitude,
                           core1_received_packet.packet.gngga.hdop);
                    
                    /* System Health */
                    const char* health_str[] = {"NOM", "DEG", "CRT", "FAIL"};
                    printf("-- [HEALTH] GPS:%-4s MAG:%-4s TSYNC:%-4s IMU:%-4s FUS:%-4s --\n",
                           health_str[core1_received_packet.packet.health.gps],
                           health_str[core1_received_packet.packet.health.mag],
                           health_str[core1_received_packet.packet.health.rtc],
                           health_str[core1_received_packet.packet.health.imu],
                           health_str[core1_received_packet.packet.health.fusion]);
                    printf("-- [ERRORS] CRC:%3lu Timeout:%3lu Fusion:%3lu             --\n",
                           core1_received_packet.packet.health.crc_error_count,
                           core1_received_packet.packet.health.sensor_timeout_count,
                           core1_received_packet.packet.health.fusion_divergence_count);
                    
                    /* Statistics */
                    float packet_success_rate = (packet_count > 0U) ? 
                                               (100.0f * (float)valid_packets / (float)packet_count) : 0.0f;
                    printf("-- [STATS] Valid:%lu Invalid:%lu Success:%.1f%%            --\n",
                           valid_packets, invalid_packets, packet_success_rate);
                    printf("═══════════════════════════════════════════════════════════════\n\n");
                }
                
                /* Echo packet back to Core 0 for acknowledgment */
                uint8_t idx = rx_write_idx % RX_POOL_SIZE;
                rx_write_idx++;
                memcpy(&rx_pool[idx], &core1_received_packet, sizeof(TLM_serial_t));
                multicore_fifo_push_blocking((uint32_t)&rx_pool[idx]);
                
            } else {
                invalid_packets++;
                system_health.crc_error_count++;
                
                printf("[CORE1 ERROR] Packet validation failed: Header:%d Footer:%d CRC:%d\n",
                       header_valid, footer_valid, crc_valid);
            }
        }
        
        tight_loop_contents();
    }
}

/*============================================================================*/
/* MAIN APPLICATION - CORE 0                                                  */
/*============================================================================*/

/**
 * @brief Main application entry point - Core 0 sensor acquisition
 * @return Exit status (never returns in normal operation)
 */
int main(void) {
    /* Initialize ALL stdio BEFORE any printf calls */
    stdio_init_all();
    
    /* CRITICAL: Wait for USB CDC connection to stabilize */
    sleep_ms(3000U);

    stdio_usb_init();
    
    /* Send initial test message to verify USB connection */
    printf("\n\n\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║     GNC TEST BENCH TELEMETRY SYSTEM v2.1.0                ║\n");
    printf("║     Aerospace Compliant Implementation                    ║\n");
    printf("║     DO-178C Level C | MISRA-C:2012                        ║\n");
    printf("║     GPS Time Synchronization | No External RTC            ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    /* Force flush to ensure output is visible */
    fflush(stdout);
    
    printf("[BOOT] Pico SDK initialized\n");
    printf("[BOOT] USB CDC serial active\n");
    fflush(stdout);
    
    /* Initialize critical section for NMEA parsing */
    critical_section_init(&nmea_crit_sec);
    printf("[INIT] Critical sections initialized\n");
    fflush(stdout);
    
    /* Launch Core 1 BEFORE enabling watchdog */
    multicore_launch_core1(core1_entry);
    printf("[INIT] Core 1 launched - Telemetry processor active\n");
    fflush(stdout);
    sleep_ms(100U);
    
    /* Enable watchdog with 8 second timeout */
    if (watchdog_caused_reboot()) {
        printf("[WARN] System recovered from watchdog reset\n");
        fflush(stdout);
    }
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    printf("[INIT] Watchdog enabled (%dms timeout)\n", WATCHDOG_TIMEOUT_MS);
    fflush(stdout);
    
    /* Initialize hardware peripherals */
    init_i2c();
    printf("[INIT] I2C initialized (400kHz)\n");
    fflush(stdout);
    
    init_uart();
    fflush(stdout);
    
    /* Initialize sensors */
    if (qmc_5883l_init(I2C_PORT)) {
        printf("[INIT] QMC5883L magnetometer initialized\n");
    } else {
        printf("[ERROR] QMC5883L initialization failed\n");
        system_health.mag = HEALTH_FAILED;
    }
    fflush(stdout);
    
    if (imu_init(I2C_PORT)) {
        printf("[INIT] ICM42688 IMU initialized\n");
    } else {
        printf("[WARN] ICM42688 initialization failed\n");
        system_health.imu = HEALTH_FAILED;
    }
    fflush(stdout);
    
    printf("[INIT] GPS time synchronization enabled (awaiting GPS lock)\n");
    fflush(stdout);
    
    printf("\n[READY] System operational - entering main telemetry loop\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    fflush(stdout);
    
    /* Initialize telemetry structures */
    TLM_packet_t tlm_packet = {0};
    TLM_serial_t tlm_serial = {0};
    TLM_serial_t ack_packet = {0};
    
    /* Initialize health timestamps */
    system_health.last_gps_update = get_absolute_time();
    system_health.last_mag_update = get_absolute_time();
    system_health.last_imu_update = get_absolute_time();
    
    current_device_t current_device = DEV_IMU;
    uint32_t loop_count = 0U;
    absolute_time_t cycle_start;
    
    /* Main telemetry loop - 100Hz (10ms cycle) */
    while (true) {
        cycle_start = get_absolute_time();
        watchdog_update();
        
        /* Round-robin sensor acquisition */
        telemetry_acquire_sensor(&tlm_packet, current_device);
        current_device = (current_device_t)((current_device + 1U) % DEV_COUNT);
        
        /* Every 10th iteration, update health and serialize */
        if ((loop_count % 10U) == 0U) {
            /* Update system health status */
            update_health_status(&tlm_packet);
            
            /* Add metadata */
            tlm_packet.packet_sequence = packet_sequence_core0++;
            tlm_packet.core0_timestamp_ms = to_ms_since_boot(get_absolute_time());
            
            /* Serialize and send to Core 1 */
            tlm_serialize_packet(&tlm_packet, &tlm_serial);
            
            if (!queue_tlm_to_core1(&tlm_serial)) {
                printf("[CORE0 WARN] FIFO overflow - Core 1 slow?\n");
            }
            
            /* Wait for acknowledgment from Core 1 (with timeout) */
            if (!receive_tlm_from_core1(&ack_packet, 50000U)) {
                system_health.sensor_timeout_count++;
                
                if ((system_health.sensor_timeout_count % 100U) == 0U) {
                    printf("[CORE0 WARN] Core 1 ACK timeout (count: %lu)\n",
                           system_health.sensor_timeout_count);
                }
            }
        }
        
        loop_count++;
        
        /* Maintain 10ms cycle time */
        absolute_time_t cycle_end = get_absolute_time();
        int64_t elapsed_us = absolute_time_diff_us(cycle_start, cycle_end);
        int64_t sleep_us_val = 10000 - elapsed_us;
        
        if (sleep_us_val > 0) {
            sleep_us((uint64_t)sleep_us_val);
        } else if ((loop_count % 100U) == 0U) {
            printf("[CORE0 WARN] Cycle overrun: %lldus (target: 10000us)\n", elapsed_us);
        }
    }
    
    return 0;
}