#include "dshot.h"

#include "bidir_dshot.pio.h"
#include "uart_rx.pio.h"
#include "uart_telemetry.h"

#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"

// ========================================
// Global Variables
// ========================================

bool dshot_initialized = false;
volatile uint32_t vcc_reading_mv = 0;
volatile bool dshot_thrusters_on = false;
volatile bool esc_board_was_off = false;

// ========================================
// Static Variables
// ========================================

/**
 * @brief Contains the offset of the DShot PIO program loaded into memory.
 * Required for command transmit calculations
 */
static uint dshot_pio_offset;

/**
 * @brief Buffer for uart telemetry being actively received
 */
struct uart_telem_buffer dshot_uart_telem_buffer = { .thruster = -1 };

/**
 * @brief Reports if the RPM should be reversed for the thruster (sent a negative value)
 */
static bool dshot_rpm_reversed[NUM_THRUSTERS] = { 0 };

// ========================================
// Inter-core shared memory
// ========================================

/**
 * @brief Protects dshot_telemetry_data when trasnferring data across cores.
 */
static spin_lock_t *dshot_telemtry_lock;

/**
 * @brief Holds UART telemetry data received from the ESC. This struct is used as a shared memory resource to transfer
 * telemetry between cores.
 *
 * @attention Should only be read or modified when dshot_telemtry_lock is held
 */
static volatile struct dshot_uart_telemetry dshot_telemetry_data[NUM_THRUSTERS] = { 0 };

// ========================================
// VCC Measurement
// ========================================

/**
 * @brief Measures the voltage divided VCC pin
 * Required so that the ESCs see a neutral signal during power up to avoid locking out the thrusters
 */
static void __time_critical_func(vcc_meas_cb)(void) {
    // Decode the measurement
    uint32_t reading_mv = ((uint32_t) (adc_fifo_get() & 0xFFF)) * VCC_CONVERSION_MULT_MV / VCC_CONVERSION_DIV_MV;
    vcc_reading_mv = reading_mv;

    // Set ESC board off if we get a reading with the ESC board off
    if (reading_mv <= ESC_POWER_THRESHOLD_MV) {
        esc_board_was_off = true;
    }
}

// ========================================
// DShot UART Telemetry RX
// ========================================

static void dshot_uart_prep_for_rx(uint thruster, bool rpm_reversed) {
    uint32_t irq = spin_lock_blocking(dshot_telemtry_lock);
    dshot_uart_telem_buffer.crc = 0;
    dshot_uart_telem_buffer.recv_index = 0;
    dshot_uart_telem_buffer.thruster = thruster;
    dshot_uart_telem_buffer.type = ESC_TELEM_FRAME;
    dshot_uart_telem_buffer.rpm_reversed = rpm_reversed;

    if (dshot_telemetry_data[thruster].missed_count < TELEM_MAX_MISSED_PACKETS) {
        dshot_telemetry_data[thruster].missed_count++;
        if (dshot_telemetry_data[thruster].total_missed < UINT16_MAX) {
            dshot_telemetry_data[thruster].total_missed++;
        }
    }
    else {
        dshot_telemetry_data[thruster].valid = false;
        if (dshot_telemetry_data[thruster].total_invalid < UINT16_MAX) {
            dshot_telemetry_data[thruster].total_invalid++;
        }
    }
    spin_unlock(dshot_telemtry_lock, irq);
}

static __always_inline uint8_t dshot_uart_crc_update(uint8_t crc, uint8_t crc_seed) {
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    return (crc_u);
}

static void __time_critical_func(dshot_uart_telem_cb)(void) {
    // Read rx fifo pending for all 4 state machines
    uint pending_buffers = DSHOT_TELEM_PIO_BLOCK->intr & 0xF;

    for (int i = 0; i < 4; i++) {
        // Try to read if pending interrupt set
        if ((pending_buffers & (1 << i)) == 0) {
            continue;
        }

        // Check if this is data we are expecting
        uint8_t data = uart_rx_program_getc(DSHOT_TELEM_PIO_BLOCK, i);
        uint32_t irq = spin_lock_blocking(dshot_telemtry_lock);
        if (dshot_uart_telem_buffer.thruster == i &&
            dshot_uart_telem_buffer.recv_index < sizeof(dshot_uart_telem_buffer.buffer)) {
            // Receive data and process it
            dshot_uart_telem_buffer.buffer.raw[dshot_uart_telem_buffer.recv_index++] = data;
            uint8_t last_crc = dshot_uart_telem_buffer.crc;
            dshot_uart_telem_buffer.crc = dshot_uart_crc_update(data, last_crc);

            // Check if frame is complete
            if (dshot_uart_telem_buffer.recv_index == sizeof(dshot_uart_telem_buffer.buffer.telem_pkt)) {
                dshot_uart_telem_buffer.thruster = -1;  // Mark as received
                struct esc_telem_pkt *pkt = &dshot_uart_telem_buffer.buffer.telem_pkt;

                if (last_crc == pkt->crc) {
                    // Successful CRC RX, load in data
                    dshot_telemetry_data[i].temperature = pkt->temperature;
                    dshot_telemetry_data[i].voltage = ((pkt->voltage_high << 8) | pkt->voltage_low);
                    dshot_telemetry_data[i].current = ((pkt->current_high << 8) | pkt->current_low);
                    dshot_telemetry_data[i].consumption = ((pkt->consumption_high << 8) | pkt->consumption_low);
                    dshot_telemetry_data[i].rpm = ((pkt->rpm_high << 8) | pkt->rpm_low);
                    dshot_telemetry_data[i].rpm_reversed = dshot_uart_telem_buffer.rpm_reversed;

                    // Clear missed packet count and mark data as valid
                    dshot_telemetry_data[i].missed_count = 0;
                    dshot_telemetry_data[i].valid = true;
                    if (dshot_telemetry_data[i].total_success < UINT16_MAX) {
                        dshot_telemetry_data[i].total_success++;
                    }
                }
            }
        }
        spin_unlock(dshot_telemtry_lock, irq);
    }
}

// ========================================
// DShot Bidir Telemetry RX
// ========================================

int8_t gcr_lookup[0x20] = {
    -1, -1,  -1,  -1,   // 0x00-0x03
    -1, -1,  -1,  -1,   // 0x04-0x07
    -1, 0x9, 0xA, 0xB,  // 0x08-0x0B
    -1, 0xD, 0xE, 0xF,  // 0x0C-0x0F
    -1, -1,  0x2, 0x3,  // 0x10-0x13
    -1, 0x5, 0x6, 0x7,  // 0x14-0x17
    -1, 0x0, 0x8, 0x1,  // 0x18-0x1B
    -1, 0x4, 0xC, -1,   // 0x1C-0x1F
};

int decode_erpm_period(uint32_t packet) {
    if (packet == 0) {
        return -1;
    }

    uint32_t gcr = (packet ^ (packet >> 1));

    int8_t crc_nibble = gcr_lookup[gcr & 0x1F];
    int8_t low_nibble = gcr_lookup[(gcr >> 5) & 0x1F];
    int8_t mid_nibble = gcr_lookup[(gcr >> 10) & 0x1F];
    int8_t high_nibble = gcr_lookup[(gcr >> 15) & 0x1F];

    if (crc_nibble < 0 || low_nibble < 0 || mid_nibble < 0 || high_nibble < 0) {
        return -1;
    }

    int8_t calculated_crc = (~(low_nibble ^ mid_nibble ^ high_nibble)) & 0xF;
    if (calculated_crc != crc_nibble) {
        return -1;
    }

    uint32_t telem_data = (high_nibble << 8) | (mid_nibble << 4) | low_nibble;
    return (telem_data & 0x1FF) << (telem_data >> 9);
}

bool dshot_get_rpm(unsigned int thruster, int *rpm_out) {
    hard_assert_if(DSHOT, !dshot_initialized);
    invalid_params_if(DSHOT, thruster >= NUM_THRUSTERS);

    int period_us = decode_erpm_period(pio_sm_get_blocking(DSHOT_PIO_BLOCK, thruster));
    if (period_us < 0) {
        return false;
    }

    // Don't convert if RPM period is 0, something is messed up
    if (period_us == 0) {
        return false;
    }
    // Check if the period is reporting no movement
    if (period_us == (0x1FF << 0x7)) {
        *rpm_out = 0;
        return true;
    }
    // Period is a valid value, convert to RPM
    else {
        // Compute RPM from period
        uint32_t erpm = (60 * 1000000) / period_us;
        int rpm = erpm / ESC_NUM_POLE_PAIRS;

        // Get the RPM sign
        if (dshot_rpm_reversed[thruster]) {
            rpm *= -1;
        }

        *rpm_out = rpm;
        return true;
    }
}

// ========================================
// DShot TX
// ========================================

static void __time_critical_func(dshot_send_command_internal)(uint thruster_index, uint throttle,
                                                              bool request_telemetry) {
    uint cmd = (throttle & 0x7FF);
    cmd <<= 1;
    if (request_telemetry) {
        cmd |= 1;
    }

    uint32_t crc = (~(cmd ^ (cmd >> 4) ^ (cmd >> 8))) & 0x0F;
    cmd <<= 4;
    cmd |= crc;

    pio_sm_put_blocking(DSHOT_PIO_BLOCK, thruster_index, (~cmd) << 16);
}

void __time_critical_func(dshot_update_thrusters)(const int16_t *throttle_values) {
    hard_assert_if(DSHOT, !dshot_initialized);

    uint dshot_cmd[NUM_THRUSTERS];

    bool thrusters_powered = false;

    // Convert the throttle values (-999 to 999) into dshot commands (48-1047 forward and 1049-2047 reverse)
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        int16_t val = throttle_values[i];

        if (val > 0 && val < 1000) {
            // Forward Direction
            // TODO: Fix this to track the zero crossing
            dshot_rpm_reversed[i] = false;
            dshot_cmd[i] = val + 48;
            thrusters_powered = true;
        }
        else if (val < 0 && val > -1000) {
            // Reverse Direction
            dshot_rpm_reversed[i] = true;
            dshot_cmd[i] = (-val) + 1048;
            thrusters_powered = true;
        }
        else {
            valid_params_if(DSHOT, val == 0);  // If an invalid dshot value got this far, that's an issue
            // 0 Value (and any other invalid values)
            dshot_cmd[i] = 0;
        }
    }

    // Update the global variable on if the thrusters are being powered
    dshot_thrusters_on = thrusters_powered;

    // Check if we need to ask for telemetry this command
    static int telem_cur_thruster = 0;
    static int telem_delay = 0;

    telem_delay++;
    if (telem_delay == TELEM_PACKET_DELAY) {
        telem_delay = 0;

        telem_cur_thruster++;
        if (telem_cur_thruster == NUM_THRUSTERS) {
            telem_cur_thruster = 0;
        }
    }

    // Reset dshot blocks for new transfer
    // This prevents weird desynchronization issues in the event an ESC responsds right as we send a new command
    for (int i = 0; i < 4; i++) {
        bidir_dshot_reset(DSHOT_PIO_BLOCK, i, dshot_pio_offset, DSHOT_RATE, nil_time);
    }

    // Send the commands to transmit, setting the telemetry bit if needed
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        if (telem_delay == 0 && telem_cur_thruster == i) {
            dshot_uart_prep_for_rx(i, dshot_rpm_reversed[i]);
            dshot_send_command_internal(i, dshot_cmd[i], true);
        }
        else {
            dshot_send_command_internal(i, dshot_cmd[i], false);
        }
    }
}

// ========================================
// Core 0 Interface Functions
// ========================================

bool dshot_get_telemetry(int thruster_num, struct dshot_uart_telemetry *telem_out) {
    invalid_params_if(DSHOT, thruster_num >= NUM_THRUSTERS);

    if (!dshot_initialized) {
        return false;
    }

    uint32_t irq = spin_lock_blocking(dshot_telemtry_lock);
    bool valid = dshot_telemetry_data[thruster_num].valid;
    if (valid) {
        *telem_out = dshot_telemetry_data[thruster_num];
    }
    else {
        telem_out->valid = false;
        telem_out->total_missed = dshot_telemetry_data[thruster_num].total_missed;
        telem_out->total_invalid = dshot_telemetry_data[thruster_num].total_invalid;
        telem_out->total_success = dshot_telemetry_data[thruster_num].total_success;
    }
    dshot_telemetry_data[thruster_num].total_missed = 0;
    dshot_telemetry_data[thruster_num].total_invalid = 0;
    dshot_telemetry_data[thruster_num].total_success = 0;
    spin_unlock(dshot_telemtry_lock, irq);

    return valid;
}

// ========================================
// Initialization
// ========================================

void dshot_init(void) {
    hard_assert_if(DSHOT, dshot_initialized);

    // Binary-info Defines for UF2
    bi_decl_if_func_used(bi_1pin_with_name(ESC1_PWM_PIN, "Thruster 1/5 DShot"));
    bi_decl_if_func_used(bi_1pin_with_name(ESC2_PWM_PIN, "Thruster 2/6 DShot"));
    bi_decl_if_func_used(bi_1pin_with_name(ESC3_PWM_PIN, "Thruster 3/7 DShot"));
    bi_decl_if_func_used(bi_1pin_with_name(ESC4_PWM_PIN, "Thruster 4/8 DShot"));
    bi_decl_if_func_used(bi_1pin_with_name(ESC1_TELEM_PIN, "Thruster 1/5 Telemetry"));
    bi_decl_if_func_used(bi_1pin_with_name(ESC2_TELEM_PIN, "Thruster 2/6 Telemetry"));
    bi_decl_if_func_used(bi_1pin_with_name(ESC3_TELEM_PIN, "Thruster 3/7 Telemetry"));
    bi_decl_if_func_used(bi_1pin_with_name(ESC4_TELEM_PIN, "Thruster 4/8 Telemetry"));
    bi_decl_if_func_used(bi_1pin_with_name(VCC_MEAS_PIN, "VCC Measurement"));

    // Initialize Telemetry Lock
    dshot_telemtry_lock = spin_lock_init(spin_lock_claim_unused(true));

    // Initialize DShot Pins
    dshot_pio_offset = pio_add_program(DSHOT_PIO_BLOCK, &bidir_dshot_program);
    pio_claim_sm_mask(DSHOT_PIO_BLOCK, 0xF);  // Claim all state machines in pio block
    bidir_dshot_program_init(DSHOT_PIO_BLOCK, 0, dshot_pio_offset, DSHOT_RATE, ESC1_PWM_PIN);
    bidir_dshot_program_init(DSHOT_PIO_BLOCK, 1, dshot_pio_offset, DSHOT_RATE, ESC2_PWM_PIN);
    bidir_dshot_program_init(DSHOT_PIO_BLOCK, 2, dshot_pio_offset, DSHOT_RATE, ESC3_PWM_PIN);
    bidir_dshot_program_init(DSHOT_PIO_BLOCK, 3, dshot_pio_offset, DSHOT_RATE, ESC4_PWM_PIN);

    // Initialize Telemetry Pins
    uint uart_offset = pio_add_program(DSHOT_TELEM_PIO_BLOCK, &uart_rx_program);
    pio_claim_sm_mask(DSHOT_TELEM_PIO_BLOCK, 0xF);  // Claim all state machines in pio block
    uart_rx_program_init(DSHOT_TELEM_PIO_BLOCK, 0, uart_offset, ESC1_TELEM_PIN, 115200);
    uart_rx_program_init(DSHOT_TELEM_PIO_BLOCK, 1, uart_offset, ESC2_TELEM_PIN, 115200);
    uart_rx_program_init(DSHOT_TELEM_PIO_BLOCK, 2, uart_offset, ESC3_TELEM_PIN, 115200);
    uart_rx_program_init(DSHOT_TELEM_PIO_BLOCK, 3, uart_offset, ESC4_TELEM_PIN, 115200);

    pio_set_irq0_source_mask_enabled(DSHOT_TELEM_PIO_BLOCK,
                                     PIO_INTR_SM0_RXNEMPTY_BITS | PIO_INTR_SM1_RXNEMPTY_BITS |
                                         PIO_INTR_SM2_RXNEMPTY_BITS | PIO_INTR_SM3_RXNEMPTY_BITS,
                                     true);

    irq_set_exclusive_handler(PIO1_IRQ_0, dshot_uart_telem_cb);
    irq_set_enabled(PIO1_IRQ_0, true);

    // Initialize VCC Measurement to detect if ESCs are powered on
    adc_init();
    adc_gpio_init(VCC_MEAS_PIN);
    adc_select_input(VCC_MEAS_PIN - 26);
    adc_set_clkdiv(48000.f);
    adc_fifo_setup(true, true, 1, false, false);
    adc_irq_set_enabled(true);
    adc_run(true);
    irq_set_exclusive_handler(ADC_IRQ_FIFO, vcc_meas_cb);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    dshot_initialized = true;
}
