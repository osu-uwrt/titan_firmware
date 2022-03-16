#include <stdio.h>

#include <riptide_msgs2/msg/pwm_stamped.h>

#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "basic_logging/logging.h"

#include "drivers/safety.h"
#include "hw/dshot.h"

#include "dshot.pio.h"

// Thruster lookup macros
// Thruster id is a value 1-8
#define THRUSTER_PIO(thruster_id) (thruster_id > 4 ? pio1 : pio0)
#define THRUSTER_SM(thruster_id) (((thruster_id-1) % 4))
#define THRUSTER_PRGM_OFFSET(thruster_id) (thruster_id > 4 ? offset_pio1 : offset_pio0)

bool dshot_initialized = false;

/**
 * @brief The alarm id of the active timeout alarm.
 * Should only be positive when an alarm is active
 */
static alarm_id_t dshot_timeout_alarm_id = 0;

/**
 * @brief Timeout for when thrusters are allowed to run.
 * If this is in the future, thrusters should not be written to.
 * Gives time for thruster startup chime to play and power rails to settle
 */
static absolute_time_t dshot_time_thrusters_allowed;

/**
 * @brief When set to true, should not let commands go to thrusters
 * Set to disable thrusters during low battery condition
 */
static bool dshot_thruster_lowbatt_disable = false;

/**
 * @brief Signals that the thruster needs to be initialized during dshot_init
 * Holds the value if kill switch is changed before dshot has been initialized
 */
static bool dshot_needs_thruster_init_on_init = false;

/**
 * @brief Sends the dshot command with the specified parameters
 * 
 * INITIALIZATION REQUIRED
 * 
 * @param thruster_id Thruster ID 1-8
 * @param pwm_value   
 * @param request_telemetry 
 * @param send_once 
 */
static inline void dshot_send_internal(uint8_t thruster_id, uint16_t throttle_value, bool request_telemetry, bool send_once) {
    invalid_params_if(DSHOT, thruster_id < 1 || thruster_id > 8);
    invalid_params_if(DSHOT, throttle_value > 0x7FF);

    uint32_t cmd = (throttle_value & 0x7FF) << 1;

    if (request_telemetry) {
        cmd |= 1;
    }

    uint32_t crc = (cmd ^ (cmd >> 4) ^ (cmd >> 8)) & 0x0F;
    cmd <<= 4;
    cmd |= crc;

    // PIO reads from MSB first, so it needs to have the top 16 bits be the data to send
    cmd <<= 16;
    if (!send_once) {
        cmd |= (1 << 15);
    }

    // The buffer should ideally never fill up. If it does unexpected results due to blocking during 
    // high priority interrputs
    if (pio_sm_is_tx_fifo_full(THRUSTER_PIO(thruster_id), THRUSTER_SM(thruster_id))) {
        LOG_ERROR("DShot PIO Buffer Stall");
        safety_raise_fault(FAULT_DSHOT_ERROR);
    }

    pio_sm_put_blocking(THRUSTER_PIO(thruster_id), THRUSTER_SM(thruster_id), cmd);
}

/**
 * @brief Sends required initialization commands to the ESCs
 * 
 * DShot must be initialized for this
 */
static void dshot_init_thrusters(void) {
    for (int i = 1; i <= 8; i++) {
        // Clear any pending commands
        pio_sm_clear_fifos(THRUSTER_PIO(i), THRUSTER_SM(i));
        // TODO: Send 10 times
        dshot_send_internal(i, 10, true, false);
    }

    dshot_time_thrusters_allowed = make_timeout_time_ms(5000);
}

void dshot_stop_thrusters(void) {
    // This command needs to be able to be called from kill switch callbacks
    // So this can be called at any point, so just ignore call if dshot is not initialized yet
    if (dshot_initialized) {
        for (int i = 1; i <= 8; i++){
            pio_sm_clear_fifos(THRUSTER_PIO(i), THRUSTER_SM(i));
            dshot_send_internal(i, 0, false, false);
        }

        if (dshot_timeout_alarm_id > 0) {
            cancel_alarm(dshot_timeout_alarm_id);
            dshot_timeout_alarm_id = 0;
        }
    }
}

void dshot_notify_physical_kill_switch_change(bool new_value) { 
    if (dshot_initialized) {
        if (new_value) {
            dshot_init_thrusters();
        }
    } else {
        dshot_needs_thruster_init_on_init = true;
    }
}

/**
 * @brief Timeout function which fires if dshot commands have not been updated in enough time
 * 
 * @param id Active alarm ID
 * @param user_data not used
 * @return int64_t Reschedule time
 */
static int64_t dshot_update_timeout(__unused alarm_id_t id, __unused void *user_data) {
    dshot_stop_thrusters();
    dshot_time_thrusters_allowed = make_timeout_time_ms(DSHOT_UPDATE_DISABLE_TIME_MS);
    safety_raise_fault(FAULT_THRUSTER_TIMEOUT);
    LOG_ERROR("Thrusters Timed Out");
    return 0;
}

void dshot_update_thrusters(riptide_msgs2__msg__PwmStamped *thruster_commands) {
    hard_assert_if(LIFETIME_CHECK, !dshot_initialized);

    // If a timeout alarm is scheduled, cancel it since there's a new update
    if (dshot_timeout_alarm_id > 0) {
        cancel_alarm(dshot_timeout_alarm_id);
        dshot_timeout_alarm_id = 0;
    }

    // Thrusters shouldn't move if they haven't initialized yet
    if (absolute_time_diff_us(dshot_time_thrusters_allowed, get_absolute_time()) < 0) {
        return;
    }

    // Thrusters shouldn't move if in low battery state
    if (dshot_thruster_lowbatt_disable) {
        return;
    }

    // Only schedule timeout alarm if thrusters will actually be enabled
    bool needs_timeout_scheduled = false;

    for (int i = 0; i < 8; i++){
        if ((thruster_commands[i] > 0 && thruster_commands[i] < 48) || thruster_commands[i] > 2047) {
            LOG_WARN("Invalid Thruster Command Sent: %d on Thruster %d", thruster_commands[i], i+1);
            safety_raise_fault(FAULT_DSHOT_ERROR);
            dshot_stop_thrusters();
            return;
        }

        if (thruster_commands[i] > 0) {
            needs_timeout_scheduled = true;
        }
        dshot_send_internal(i+1, thruster_commands[i], false, false);
    }

    if (needs_timeout_scheduled) {
        dshot_timeout_alarm_id = add_alarm_in_ms(DSHOT_MIN_UPDATE_RATE_MS, &dshot_update_timeout, NULL, true);
        hard_assert(dshot_timeout_alarm_id > 0);
    }
}

void dshot_set_lowbatt(bool in_lowbatt_state) {
    dshot_thruster_lowbatt_disable = in_lowbatt_state;
    if (in_lowbatt_state) {
        dshot_stop_thrusters();
    }
}

#define init_thruster_pio(thruster_id) bi_decl_if_func_used(bi_1pin_with_name(THRUSTER_##thruster_id##_PIN, "Thruster " #thruster_id)); \
                                       dshot_program_init(THRUSTER_PIO(thruster_id), THRUSTER_SM(thruster_id), THRUSTER_PRGM_OFFSET(thruster_id), \
                                                          clock_get_hz(clk_sys) / DSHOT_RATE(300), THRUSTER_##thruster_id##_PIN)
void dshot_init(void) {
    hard_assert_if(LIFETIME_CHECK, dshot_initialized);
    uint offset_pio0 = pio_add_program(pio0, &dshot_program);
    uint offset_pio1 = pio_add_program(pio1, &dshot_program);

    init_thruster_pio(1);
    init_thruster_pio(2);
    init_thruster_pio(3);
    init_thruster_pio(4);
    init_thruster_pio(5);
    init_thruster_pio(6);
    init_thruster_pio(7);
    init_thruster_pio(8);

    if (dshot_needs_thruster_init_on_init){
        dshot_init_thrusters();
    }
    
    dshot_initialized = true;

    dshot_stop_thrusters();
}