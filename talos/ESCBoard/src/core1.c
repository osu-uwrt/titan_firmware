#include "core1.h"

#include "Thruster_Controller/ThrusterController.h"
#include "dshot.h"
#include "safety_interface.h"

#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

// ========================================
// Target RPM Shared Memory
// ========================================

/**
 * @brief Spin lock to protect Target RPM state when transferring across cores
 */
static spin_lock_t *target_rpm_lock;

/**
 * @brief The target RPM from ROS for the controller to run.
 *
 * @attention Can only be modified or read when target_rpm_lock is held.
 */
static volatile int16_t target_rpm[NUM_THRUSTERS] = { 0 };

/**
 * @brief The target RPM is set to the raw dshot command.
 *
 * This is useful when tuning the controller feed-forward, as it bypasses the controller and issues raw dshot commands.
 *
 * @attention Can only be modified or read when target_rpm_lock is held.
 */
static volatile bool target_rpm_is_raw_cmd = false;

/**
 * @brief The time after which the target RPM command is considered stale, and the thrusters should be disabled.
 * Prevents runaway robots.
 *
 * @attention Can only be modified or read when target_rpm_lock is held.
 */
static volatile absolute_time_t target_rpm_expiration = { 0 };

/**
 * @brief Incremented every control loop tick. Can be used to compute controller rate
 *
 * @attention Can only modified or read when target_rpm_lock is held
 */
static volatile unsigned int control_loop_tick_cnt;

// ========================================
// Current RPM Shared Memory
// ========================================

/**
 * @brief Spin lock to prevent Current RPM state when transferring across cores
 */
static spin_lock_t *current_rpm_lock;

/**
 * @brief Contains the last received RPM data from the ESC
 *
 * @attention Can only be modified or read when current_rpm_lock is held.
 */
static volatile struct dshot_rpm_telem {
    int16_t rpm;
    uint8_t missed_count;
    bool valid;
} current_rpm[NUM_THRUSTERS];

/**
 * @brief Contains the last command sent to the thrusters.
 *
 * @attention Can only be modified or read when current_rpm_lock is held.
 */
static volatile int16_t current_thruster_cmd[NUM_THRUSTERS] = { 0 };

// ========================================
// Core 1 Main Loop
// ========================================

#define FIFO_REQ_VALUE_WIDTH 24

/**
 * @brief The encoded command sent over the multicore FIFO. The FIFO will be processed at the start of every control
 * loop. This prevents race conditions of modifying the variables directly.
 */
typedef union sio_fifo_req {
    struct __packed {
        enum __packed param_type {
            CONTROLLER_PARAM_P_GAIN,
            CONTROLLER_PARAM_I_GAIN,
            CONTROLLER_PARAM_I_BOUND,
            CONTROLLER_PARAM_HARD_LIMIT,
            CONTROLLER_PARAM_MIN_CMD,
        } type;
        uint32_t value : FIFO_REQ_VALUE_WIDTH;
    };
    uint32_t raw;
} sio_fifo_req_t;
static_assert(sizeof(sio_fifo_req_t) == sizeof(uint32_t), "FIFO Command did not pack properly");

/**
 * @brief Main loop for the thruster control loop. Entry point for core 1.
 */
static void __time_critical_func(core1_main)() {
    // TODO: Add in crash handling logic to core 1

    dshot_init();

    // ========================================
    // Control Loop State Variables
    // ========================================

    // The throttle command computed on the last tick
    int16_t throttle_commands[NUM_THRUSTERS] = { 0 };
    // The time of the last controller tick. Used to compute I gains
    absolute_time_t last_tick = nil_time;
    // The time after which the ESCs are considered to be started ujp
    absolute_time_t esc_startup_finished = make_timeout_time_ms(ESC_WAKEUP_DELAY_MS);
    // If the RPM command expires, this will be set to DSHOT_UPDATE_DISABLE_TIME_MS to prevent thruster jitter
    absolute_time_t command_timeout_penalty = nil_time;
    // Holds the state of the thruster controller
    thruster_controller_state_t controller_state[NUM_THRUSTERS] = { 0 };
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        thruster_controller_init_defaults(&controller_state[i]);
    }

    // ========================================
    // Main Control Loop
    // ========================================

    bool in_raw_mode = false;

    while (1) {
        // Time difference used for I gains, computed later in control loop if I gains should be enabled in this tick
        int64_t time_difference = 0;

        // ========================================
        // Phase 1: Copy in external commands
        // ========================================

        // Make copy of target request under lock
        // Prevents race conditions if Core 0 is modifying the state while we're in the middle of processing
        uint32_t irq = spin_lock_blocking(target_rpm_lock);
        bool target_rpm_expired = time_reached(target_rpm_expiration);
        bool target_rpm_is_raw_cmd_cached = target_rpm_is_raw_cmd;
        int16_t target_rpm_cached[NUM_THRUSTERS];
        for (int i = 0; i < NUM_THRUSTERS; i++) {
            target_rpm_cached[i] = target_rpm[i];
        }

        // Update tick profiler
        if (control_loop_tick_cnt < UINT32_MAX) {
            control_loop_tick_cnt++;
        }
        spin_unlock(target_rpm_lock, irq);

        // Handle any commands that have queued in the multicore fifo since the last tick
        // This updates the controller tunings at a deterministic point of the control loop
        if (multicore_fifo_rvalid()) {
            sio_fifo_req_t req = { .raw = multicore_fifo_pop_blocking() };
            if (req.type == CONTROLLER_PARAM_P_GAIN) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].Pgain = req.value;
                }
            }
            else if (req.type == CONTROLLER_PARAM_I_GAIN) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].Igain = req.value;
                }
            }
            else if (req.type == CONTROLLER_PARAM_I_BOUND) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].Ibound = req.value;
                }
            }
            else if (req.type == CONTROLLER_PARAM_HARD_LIMIT) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].hardLimit = req.value;
                }
            }
            else if (req.type == CONTROLLER_PARAM_MIN_CMD) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].minCommand = req.value;
                }
            }
            // Zero controller after adjusting tunings
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                thruster_controller_zero(&controller_state[i]);
            }
        }

        if (target_rpm_is_raw_cmd_cached && !in_raw_mode) {
            in_raw_mode = true;
            safety_raise_fault(FAULT_RAW_MODE);
        }
        else if (!target_rpm_is_raw_cmd_cached && in_raw_mode) {
            in_raw_mode = false;
            safety_lower_fault(FAULT_RAW_MODE);
        }

        // ========================================
        // Phase 2: Control Loop Overrides (Kill/Raw Mode)
        // ========================================

        // Capture if the ADC read callback saw that the ESC board turned off
        bool board_off_cached = esc_board_was_off;
        esc_board_was_off = false;

        // Check for conditions which require the controller to be disabled for safety reasons
        //  - Kill switch is pulled
        //  - ESC Board does not have power (or it power cycled)
        //  - RPM command has gone stale
        //  - ESC startup delay (ESC board received power, but has yet to chime)
        //  - RPM timeout lockout (After RPM times out, don't accept any commands for a brief period to prevent jitter)

        if (safety_kill_get_asserting_kill() || target_rpm_expired || !time_reached(esc_startup_finished) ||
            !time_reached(command_timeout_penalty) || !esc_board_on || board_off_cached) {
            // Clear command and controller state. When the controller starts up again, it should be starting the same
            // as though it was a clean boot.
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                throttle_commands[i] = 0;
                thruster_controller_zero(&controller_state[i]);
            }
            last_tick = nil_time;

            // If the reason for entering this shutdown code was the target rpm expiration, raise a fault
            // This logic works as dshot_thrusters_on is only true if this is the first control loop since the
            // controller was disabled. So, if target RPM command is expired, and its the first time entering the
            // disable logic, then the reason it was disabled must be for the target RPM expiring.
            if (dshot_thrusters_on && target_rpm_expired) {
                safety_raise_fault(FAULT_RPM_CMD_TIMEOUT);
                command_timeout_penalty = make_timeout_time_ms(DSHOT_UPDATE_DISABLE_TIME_MS);
            }

            // If the ESC board is currently not on, or we saw that a previous reading registered that it was off,
            // set the startup delay. Once the ESC board, comes back on, this timeout will prevent commands for the
            // startup delay.
            if (!esc_board_on || esc_board_was_off) {
                esc_startup_finished = make_timeout_time_ms(ESC_WAKEUP_DELAY_MS);
            }
        }

        // If the controller is in raw mode, ignore the controller and send the throttle commands to it directly
        else if (target_rpm_is_raw_cmd_cached) {
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                int16_t throttle = target_rpm_cached[i];
                if (throttle >= -999 && throttle <= 999) {
                    // Set throttle command to target RPM if valid dshot command
                    throttle_commands[i] = throttle;
                }
                else {
                    // Zero out throttle if out of range
                    throttle_commands[i] = 0;
                }
                // Disable the controller
                target_rpm_cached[i] = 0;
                thruster_controller_zero(&controller_state[i]);
            }
            last_tick = nil_time;
        }

        // Normal control path
        else {
            // If controller hasn't been disabled, compute tick difference
            // Compute the time delta between sending commands to the controller
            absolute_time_t current_time = get_absolute_time();
            if (!is_nil_time(last_tick)) {
                // Only enable I gains if we have a proper time difference from the last control loop
                time_difference = absolute_time_diff_us(last_tick, current_time);
            }
            last_tick = current_time;
        }

        // ========================================
        // Phase 3: Dshot Command Transmit
        // ========================================

        // Send command, and compute min and max update timestamps
        // Note this command comes from the last controller tick (unless its been killed)
        dshot_update_thrusters(throttle_commands);
        // Time at which we are okay to send the next packet. Sending too fast will cause the ESC to drop the message
        absolute_time_t min_frame_time = make_timeout_time_us(DSHOT_MIN_FRAME_TIME_US);

        // Save last command to be sent later
        int16_t last_command[NUM_THRUSTERS];
        for (int i = 0; i < NUM_THRUSTERS; i++) {
            last_command[i] = throttle_commands[i];
        }

        // ========================================
        // Phase 4: Dshot RPM Receive
        // ========================================

        // Create mask of thrusters waiting for response
        uint8_t waiting = (1 << NUM_THRUSTERS) - 1;

        // Wait for all controllers to run, or the max rx timeout lapses
        // The RX timeout is in the event one ESC stops responding, the control loop won't lock up
        int rpm[NUM_THRUSTERS];
        bool rpm_valid[NUM_THRUSTERS] = { 0 };
        // Serviced keeps track if the control loop was ticked the inner while loop, prevents a slow controller from
        // starving other thrusters, if say thruster 3 was the first received, but processing took longer than
        // min_frame_time, it'll tick one last time to see we received 1 during that time and process it before breaking
        // out of the loop.

        // Alex put this here - please make pretty

        // 1 2 3 4
        int8_t thrusterSigns[NUM_THRUSTERS] = { -1, 1, 1, 1 };
        if (gpio_get(BOARD_DET_PIN)) {
            // 5 6 7 8
            thrusterSigns[0] = -1;
            thrusterSigns[1] = 1;
            thrusterSigns[2] = 1;
            thrusterSigns[3] = 1;
        }

        bool serviced = false;
        while (waiting != 0 && (!time_reached(min_frame_time) || serviced)) {
            serviced = false;
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                if (dshot_rpm_available(i)) {
                    // Only tick controller if we can decode, and it hasn't been ticked yet this loop
                    // If for whatever reason, there is noise on line and PIO thinks its seen RPM twice, it won't
                    // affect the controller
                    if (dshot_get_rpm(i, &rpm[i]) && (waiting & (1 << i))) {
                        rpm_valid[i] = true;
                        // If the decode was successful, tick the controller

                        throttle_commands[i] = thruster_controller_tick(
                            &controller_state[i], target_rpm_cached[i] * thrusterSigns[i], rpm[i], time_difference);
                    }
                    // Clear waiting, even on unsuccessful response
                    waiting &= ~(1 << i);
                    serviced = true;
                }
            }
        }

        // ========================================
        // Phase 5: RPM Post Processing
        // ========================================

        irq = spin_lock_blocking(current_rpm_lock);
        for (int i = 0; i < NUM_THRUSTERS; i++) {
            // Writeback RPM data if valid
            if (rpm_valid[i]) {
                if (rpm[i] > INT16_MAX) {
                    current_rpm[i].rpm = INT16_MAX;
                }
                else if (rpm[i] < INT16_MIN) {
                    current_rpm[i].rpm = INT16_MIN;
                }
                else {
                    current_rpm[i].rpm = (int16_t) rpm[i];
                }
                current_rpm[i].missed_count = 0;
                current_rpm[i].valid = true;
            }
            // Keep track if RPM not valid
            else {
                if (current_rpm[i].missed_count < TELEM_MAX_MISSED_PACKETS) {
                    // Increment missed count if we couldn't read it (and only in if statement to prevent overflows)
                    current_rpm[i].missed_count++;
                }
                else {
                    // If we've missed too many, mark RPM as invalid
                    current_rpm[i].rpm = 0;
                    current_rpm[i].valid = false;

                    // Additionally, clear the last command
                    // We let the thruster miss a few telemetry packets and just run the an old command, but if we miss
                    // too many, we need to clear the command since the control loop hasn't been able refresh in a while
                    throttle_commands[i] = 0;
                }
            }
            current_thruster_cmd[i] = last_command[i];
        }
        spin_unlock(current_rpm_lock, irq);

        // ========================================
        // Phase 6: Dshot Min Delay Frame Wait
        // ========================================

        // If the control loop finishes before the minimum frame time, wait before sending the next command
        if (!time_reached(min_frame_time)) {
            sleep_until(min_frame_time);
        }
    }
}

// ========================================
// Core 0 Interface Functions
// ========================================

void core1_init(void) {
    target_rpm_lock = spin_lock_init(spin_lock_claim_unused(true));
    current_rpm_lock = spin_lock_init(spin_lock_claim_unused(true));
    multicore_launch_core1(core1_main);
}

void core1_update_target_rpm(const int16_t *rpm) {
    // Compute expiration outside of spin lock to avoid unnecessary work in critical section.
    absolute_time_t expiration = make_timeout_time_ms(DSHOT_MIN_UPDATE_RATE_MS);

    // Update shared variables under lock
    uint32_t irq = spin_lock_blocking(target_rpm_lock);
    target_rpm_expiration = expiration;
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        target_rpm[i] = rpm[i];
    }
    spin_unlock(target_rpm_lock, irq);
}

float core1_get_loop_rate(void) {
    uint32_t irq = spin_lock_blocking(target_rpm_lock);
    unsigned int last_tick_count = control_loop_tick_cnt;
    control_loop_tick_cnt = 0;
    spin_unlock(target_rpm_lock, irq);

    static absolute_time_t last_rate_read = { 0 };
    absolute_time_t now = get_absolute_time();
    float time_diff_us = absolute_time_diff_us(last_rate_read, now);
    last_rate_read = now;

    return ((float) last_tick_count) * 1.0E6 / time_diff_us;
}

void core1_set_raw_mode(bool raw_mode) {
    uint32_t irq = spin_lock_blocking(target_rpm_lock);
    target_rpm_is_raw_cmd = raw_mode;
    spin_unlock(target_rpm_lock, irq);
}

bool core1_get_current_rpm(int thruster_num, int16_t *rpm_out, int16_t *cmd_out) {
    invalid_params_if(DSHOT, thruster_num >= NUM_THRUSTERS);

    uint32_t irq = spin_lock_blocking(current_rpm_lock);
    bool valid = current_rpm[thruster_num].valid;
    if (valid) {
        if (rpm_out)
            *rpm_out = current_rpm[thruster_num].rpm;
    }
    if (cmd_out)
        *cmd_out = current_thruster_cmd[thruster_num];
    spin_unlock(current_rpm_lock, irq);
    return valid;
}

bool core1_set_p_gain(int32_t value) {
    if (value < 0 || value >= (1 << FIFO_REQ_VALUE_WIDTH))
        return false;
    sio_fifo_req_t req = { .type = CONTROLLER_PARAM_P_GAIN, .value = value };
    multicore_fifo_push_blocking(req.raw);
    return true;
}

bool core1_set_i_gain(int32_t value) {
    if (value < 0 || value >= (1 << FIFO_REQ_VALUE_WIDTH))
        return false;
    sio_fifo_req_t req = { .type = CONTROLLER_PARAM_I_GAIN, .value = value };
    multicore_fifo_push_blocking(req.raw);
    return true;
}

bool core1_set_i_bound(int32_t value) {
    if (value < 0 || value >= (1 << FIFO_REQ_VALUE_WIDTH))
        return false;
    sio_fifo_req_t req = { .type = CONTROLLER_PARAM_I_BOUND, .value = value };
    multicore_fifo_push_blocking(req.raw);
    return true;
}

bool core1_set_hard_limit(int32_t value) {
    if (value < 0 || value >= (1 << FIFO_REQ_VALUE_WIDTH))
        return false;
    sio_fifo_req_t req = { .type = CONTROLLER_PARAM_HARD_LIMIT, .value = value };
    multicore_fifo_push_blocking(req.raw);
    return true;
}

bool core1_set_min_command(int32_t value) {
    if (value < 0 || value >= (1 << FIFO_REQ_VALUE_WIDTH))
        return false;
    sio_fifo_req_t req = { .type = CONTROLLER_PARAM_MIN_CMD, .value = value };
    multicore_fifo_push_blocking(req.raw);
    return true;
}
