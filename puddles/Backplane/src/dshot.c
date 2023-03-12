#include <stdio.h>

#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "basic_logger/logging.h"

#include "safety_interface.h"
#include "dshot.h"
#include "bidir_dshot.pio.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "dshot"

// Sanity check parameters
static_assert(bidir_dshot_min_frame_period_us(DSHOT_RATE) <= DSHOT_TX_RATE_US, "DShot TX Rate must be greater than minimum frame time");

#define INDEX_TO_PIO(thruster_index) (((thruster_index)/4) == 0 ? pio0 : (((thruster_index)/4) == 1 ? pio1 : NULL))
#define INDEX_TO_SM(thruster_index) ((thruster_index) % 4)

// ========================================
// Global Variables
// ========================================

bool dshot_initialized = false;
struct dshot_rpm_telemetry dshot_rpm_data[NUM_THRUSTERS] = {0};
bool dshot_rpm_reversed[NUM_THRUSTERS] = {0};

// ========================================
// Static Variables
// ========================================

/**
 * @brief Timeout for when thrusters are allowed to run.
 * If this is in the future, thrusters should not be written to.
 * Gives time for thruster startup chime to play and power rails to settle
 */
static absolute_time_t dshot_time_thrusters_allowed;

/**
 * @brief The time upon which the next tx can occur.
 * This is due to the minimum frame spacing requirements for DShot
 */
static absolute_time_t dshot_next_allowed_frame_tx;

/**
 * @brief The time upon which the last published command will time out
 * In the event the controller dies, this should time out and prevent a runaway robot
 */
static absolute_time_t dshot_command_timeout;

/**
 * @brief Contains the offset of the DShot PIO program loaded into memory.
 * Required for command transmit calculations
 */
static uint dshot_pio_offset;

/**
 * @brief Bool reporting if the ESCs are commanding a non-neutral value.
 * If this is true, the thrusters are considered to be on (used for error reporting timeouts)
 */
static bool dshot_thrusters_on = false;

/**
 * @brief The cached commands to send for dshot
 */
static uint dshot_thruster_cmds[NUM_THRUSTERS] = {0};

/**
 * @brief Sets the time thrusters are allowed to the latest possible value
 * If time_allowed is greater than the current allowed time, it will set
 * If not it will keep the previous value
 *
 * @param time_allowed The time to set the allowed time to
 */
static inline void dshot_set_time_thrusters_allowed(absolute_time_t time_allowed) {
    uint32_t prev_interrupts = save_and_disable_interrupts();

    if (absolute_time_diff_us(dshot_time_thrusters_allowed, time_allowed) > 0) {
        dshot_time_thrusters_allowed = time_allowed;
    }

    restore_interrupts(prev_interrupts);
}

static inline void dshot_clear_command_state(void) {
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        dshot_thruster_cmds[i] = 0;
    }
    dshot_thrusters_on = false;
    dshot_command_timeout = nil_time;
}

// ========================================
// DShot Bidir Telemetry RX
// ========================================

int8_t gcr_lookup[0x20] = {
    -1,  -1,  -1,  -1,  // 0x00-0x03
    -1,  -1,  -1,  -1,  // 0x04-0x07
    -1, 0x9, 0xA, 0xB,  // 0x08-0x0B
    -1, 0xD, 0xE, 0xF,  // 0x0C-0x0F
    -1,  -1, 0x2, 0x3,  // 0x10-0x13
    -1, 0x5, 0x6, 0x7,  // 0x14-0x17
    -1, 0x0, 0x8, 0x1,  // 0x18-0x1B
    -1, 0x4, 0xC,  -1,  // 0x1C-0x1F
};

int __time_critical_func(decode_erpm_period)(uint32_t packet) {
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

void __time_critical_func(dshot_telem_cb)(PIO pio_hw) {
    uint thruster_base = 4 * pio_get_index(pio_hw);
    for (int i = 0; i < 4; i++) {
        if (pio_sm_get_rx_fifo_level(pio_hw, i) > 0) {
            int period_us = decode_erpm_period(pio_sm_get_blocking(pio_hw, i));
            if (period_us >= 0) {
                dshot_rpm_data[thruster_base + i].rpm_period_us = period_us;
                dshot_rpm_data[thruster_base + i].valid = true;
                dshot_rpm_data[thruster_base + i].missed_count = 0;
            }
        }
    }
}

void __time_critical_func(dshot_telem_pio0_cb)(void) {
    profiler_push(PROFILER_DSHOT_RX_IRQ0);
    dshot_telem_cb(pio0);
    profiler_pop(PROFILER_DSHOT_RX_IRQ0);
}

void __time_critical_func(dshot_telem_pio1_cb)(void) {
    profiler_push(PROFILER_DSHOT_RX_IRQ1);
    dshot_telem_cb(pio1);
    profiler_pop(PROFILER_DSHOT_RX_IRQ1);
}


// ========================================
// DShot TX
// ========================================

void __time_critical_func(dshot_send_command_internal)(uint thruster_index, uint throttle, bool request_telemetry) {
    uint cmd = (throttle & 0x7FF);
    cmd <<= 1;
    if (request_telemetry) {
        cmd |= 1;
    }

    uint32_t crc = (~(cmd ^ (cmd >> 4) ^ (cmd >> 8))) & 0x0F;
    cmd <<= 4;
    cmd |= crc;

    if (dshot_rpm_data[thruster_index].missed_count < TELEM_MAX_MISSED_PACKETS){
        dshot_rpm_data[thruster_index].missed_count++;
    } else {
        dshot_rpm_data[thruster_index].valid = false;
    }

    pio_sm_put_blocking(INDEX_TO_PIO(thruster_index), INDEX_TO_SM(thruster_index), (~cmd) << 16);
}

void __time_critical_func(dshot_transmit_timer_cb)(uint hardware_alarm_num) {
    profiler_push(PROFILER_DSHOT_TX_IRQ);

    // Raise fault if thrusters timed out
    if (time_reached(dshot_command_timeout) && dshot_thrusters_on) {
        safety_raise_fault(FAULT_THRUSTER_TIMEOUT);
        dshot_set_time_thrusters_allowed(make_timeout_time_ms(DSHOT_UPDATE_DISABLE_TIME_MS));
    }

    if (time_reached(dshot_command_timeout) || safety_kill_get_asserting_kill() ||
            !time_reached(dshot_time_thrusters_allowed)) {
        dshot_clear_command_state();
    }

    // Loop until the timer is successfully scheduled
    // If for whatever reason a high latency, higher-priority IRQ fires during scheduling, it could miss, breaking the timer
    do {
        for (int i = 0; i < NUM_THRUSTERS; i++) {
            dshot_send_command_internal(i, dshot_thruster_cmds[i], false);
        }
        dshot_next_allowed_frame_tx = make_timeout_time_us(bidir_dshot_min_frame_period_us(DSHOT_RATE));
    } while(hardware_alarm_set_target(hardware_alarm_num, make_timeout_time_us(DSHOT_TX_RATE_US)));

    profiler_pop(PROFILER_DSHOT_TX_IRQ);
}

// ========================================
// Exported Functions
// ========================================

void dshot_stop_thrusters(void) {
    // This command needs to be able to be called from kill switch callbacks
    // So this can be called at any point, so just ignore call if dshot is not initialized yet
    if (dshot_initialized) {
        dshot_clear_command_state();
        // Since this can be called at *any* time, trying to cancel/restart alarms could cause
        // nasty race conditions.

        // So to avoid this, this will just clear the command state
        // Upon the next dshot TX, it will see this data and stop the thrusters
        // Due to the strict update rates for DShot, this should occur relatively quickly (<10ms)
        // In that case, it should be fine to just clear and wait for repeated transmit
    }
}

void dshot_notify_physical_kill_switch_change(bool asserting_kill) {
    static bool last_asserting_kill = true;
    if (last_asserting_kill) {
        dshot_set_time_thrusters_allowed(make_timeout_time_ms(DSHOT_WAKEUP_DELAY_MS));
    }

    last_asserting_kill = asserting_kill;
}

void dshot_update_thrusters(const int16_t *throttle_values) {
    hard_assert_if(DSHOT, !dshot_initialized);

    // Don't run thrusters if kill is being asserted
    if (safety_kill_get_asserting_kill()) {
        return;
    }

    // Thrusters shouldn't move if they have a timeout applied
    if (!time_reached(dshot_time_thrusters_allowed)) {
        return;
    }

    // Don't send a command if one is still currently transmitting
    if (!time_reached(dshot_next_allowed_frame_tx)) {
        return;
    }

    // Cancel pending dshot transmission alarm
    cancel_alarm(dshot_hardware_alarm_num);

    // Clear the thrusters on
    dshot_thrusters_on = false;

    for (int i = 0; i < NUM_THRUSTERS; i++){
        int16_t val = throttle_values[i];

        if (val == 0) {
            dshot_thruster_cmds[i] = 0;
        } else if (val > 0 && val < 1000) {
            // TODO: Fix this to track the zero crossing
            dshot_rpm_reversed[i] = false;
            dshot_thrusters_on = true;
            dshot_thruster_cmds[i]= val + 48;
        } else if (val < 0 && val > -1000) {
            dshot_rpm_reversed[i] = true;
            dshot_thrusters_on = true;
            dshot_thruster_cmds[i]= (-val) + 1048;
        } else {
            LOG_WARN("Invalid Thruster Command Sent: %d", val);
            safety_raise_fault(FAULT_ROS_BAD_COMMAND);
            dshot_clear_command_state();
            break;
        }
    }

    // Start immediate retransmission
    for (int i = 0; i < 8; i++) {
        bidir_dshot_reset(INDEX_TO_PIO(i), INDEX_TO_SM(i), dshot_pio_offset, DSHOT_RATE, dshot_next_allowed_frame_tx);
    }

    dshot_command_timeout = make_timeout_time_ms(DSHOT_MIN_UPDATE_RATE_MS);
    dshot_transmit_timer_cb(dshot_hardware_alarm_num);
}

void dshot_init(void) {
    hard_assert_if(DSHOT, dshot_initialized);

    // Initialize DShot Pins
    dshot_pio_offset = pio_add_program(pio0, &bidir_dshot_program);
    pio_add_program_at_offset(pio1, &bidir_dshot_program, dshot_pio_offset);

    // I know... Macros.. Sue me, but it's way better than doing this all by hand IMO
    #define INIT_THRUSTER(num) do { \
       bi_decl_if_func_used(bi_1pin_with_name(THRUSTER_##num##_PIN, "Thruster " __XSTRING(num) " DShot")); \
       bidir_dshot_program_init(INDEX_TO_PIO(num-1), INDEX_TO_SM(num-1), dshot_pio_offset, DSHOT_RATE, THRUSTER_##num##_PIN); \
    } while(0)

    INIT_THRUSTER(1);
    INIT_THRUSTER(2);
    INIT_THRUSTER(3);
    INIT_THRUSTER(4);
    INIT_THRUSTER(5);
    INIT_THRUSTER(6);
    INIT_THRUSTER(7);
    INIT_THRUSTER(8);

    pio_set_irq0_source_mask_enabled(pio0,
        PIO_INTR_SM0_RXNEMPTY_BITS | PIO_INTR_SM1_RXNEMPTY_BITS |
        PIO_INTR_SM2_RXNEMPTY_BITS | PIO_INTR_SM3_RXNEMPTY_BITS,
        true);
    pio_set_irq0_source_mask_enabled(pio1,
        PIO_INTR_SM0_RXNEMPTY_BITS | PIO_INTR_SM1_RXNEMPTY_BITS |
        PIO_INTR_SM2_RXNEMPTY_BITS | PIO_INTR_SM3_RXNEMPTY_BITS,
        true);

    irq_set_exclusive_handler(PIO0_IRQ_0, dshot_telem_pio0_cb);
    irq_set_exclusive_handler(PIO1_IRQ_0, dshot_telem_pio1_cb);
    // irq_set_enabled(PIO0_IRQ_0, true);
    // irq_set_enabled(PIO1_IRQ_0, true);

    // Configure hardware alarm for DShot Command Scheduling
    hardware_alarm_claim(dshot_hardware_alarm_num);
    hardware_alarm_set_callback(dshot_hardware_alarm_num, dshot_transmit_timer_cb);

    // Start the command scheduling
    dshot_transmit_timer_cb(dshot_hardware_alarm_num);

    dshot_initialized = true;
}