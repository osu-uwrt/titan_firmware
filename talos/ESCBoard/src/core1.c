#include "core1.h"

#include "Thruster_Controller/ThrusterController.h"
#include "dshot.h"
#include "safety_interface.h"

#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

// ========================================
// Shared Memory Definitions
// ========================================

/**
 * @brief Shared memory for sending commands from core 0 to core 1.
 *
 * @attention This can only be modified or read when lock is held
 */
static volatile struct core1_cmd_shared_mem {
    /**
     * @brief Spin lock to protect command state transferring across cores
     */
    spin_lock_t *lock;

    /**
     * @brief The target RPM for the controller. Provided by ROS.
     */
    int16_t rpm[NUM_THRUSTERS];

    /**
     * @brief he time after which the target RPM command is considered stale, and the thrusters should be disabled.
     * Prevents runaway robots.
     */
    absolute_time_t rpm_expiration;
} target_req = { 0 };

/**
 * @brief Shared memory for sending telemetry from core 1 back to core 0.
 *
 * @attention This can only be modified or read when lock is held
 */
static volatile struct core1_telem_shared_mem {
    /**
     * @brief Spin lock to protect telemetry state transferring across cores
     */
    spin_lock_t *lock;

    /**
     * @brief Incremented every control loop tick. Can be used to compute controller rate
     */
    uint16_t controller_tick_cnt;

    /**
     * @brief Bitwise flags for reasons that the controller could be disabled.
     * If this is non-zero, all commands are ignored and the ESCs will be sent a zero command
     */
    uint8_t disabled_flags;

    /**
     * @brief Contains the last received RPM data from the ESC
     */
    struct thruster_state {
        int16_t cmd;               // The command currently being sent to the thruster
        int16_t rpm;               // Last captured RPM feedback from dshot (only valid if rpm_valid true)
        bool rpm_valid;            // True if rpm field is considered valid
        bool thruster_ready;       // Set to false if the
        uint8_t rpm_missed_count;  // Number of missed RPM packets. Used to compute rpm_valid
        uint16_t ticks_missed;     // Total # controller ticks this thruster missed (but still alive)
        uint16_t ticks_offline;    // Total # controller ticks this thruster was taken offline due to no RPM feedback
    } thruster[NUM_THRUSTERS];
} telem_state = { 0 };

// ========================================
// Core 1 Main Loop
// ========================================

// Must be 24-bits as the enum is 8-bit when packing into a 32-bit word
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
            CONTROLLER_PARAM_INVERT_MASK,
            CONTROLLER_PARAM_RAW_MODE,

            // Must be last
            CONTROLLER_PARAM__COUNT
        } type;
        uint32_t value : FIFO_REQ_VALUE_WIDTH;
    };
    uint32_t raw;
} sio_fifo_req_t;
static_assert(sizeof(sio_fifo_req_t) == sizeof(uint32_t), "FIFO Command did not pack properly");

/**
 * @brief Contains the controller state. Must be in static context so we don't fill up the stack with the variable.
 *
 * @attention This should only be accessed from core1_main
 */
static thruster_controller_state_t controller_state[NUM_THRUSTERS] = { 0 };

/**
 * @brief Main loop for the thruster control loop. Entry point for core 1.
 */
static void __time_critical_func(core1_main)() {
    dshot_init();

    // ========================================
    // Control Loop State Variables
    // ========================================

    // The target RPM is interpreted as raw dshot commands. Useful when tuning the feed-forward controller
    bool in_raw_mode = false;
    // The throttle command computed on the last tick
    int16_t throttle_commands[NUM_THRUSTERS] = { 0 };
    // The time of the last controller tick. Used to compute I gains
    absolute_time_t last_tick = nil_time;
    // The time after which the ESCs are considered to be started up
    absolute_time_t esc_startup_finished = make_timeout_time_ms(ESC_POWERUP_DELAY_MS);
    // If the RPM command expires, this will be set to DSHOT_UPDATE_DISABLE_TIME_MS to prevent thruster jitter
    absolute_time_t command_timeout_penalty = nil_time;
    // Bitwise array to keep track of which parameters of the controller still have to be configured
    // Keeps track of what params are needed before the controller has been fully configured by ROS
    // The controller will be kept disabled until all parameters have been set at least once
    static_assert(CONTROLLER_PARAM__COUNT < 32, "Cannot fit all controller params into 32-bit int");
    uint32_t controller_missing_params = (1 << CONTROLLER_PARAM__COUNT) - 1;

    // ========================================
    // Main Control Loop
    // ========================================

    while (1) {
        // Tick Safety
        safety_core1_checkin();

        // Time difference used for I gains, computed later in control loop if I gains should be enabled in this tick
        int64_t time_difference = 0;

        // ========================================
        // Phase 1: Copy in external commands
        // ========================================

        // Make copy of target request under lock
        // Prevents race conditions if Core 0 is modifying the state while we're in the middle of processing
        uint32_t irq = spin_lock_blocking(target_req.lock);
        bool target_rpm_expired = time_reached(target_req.rpm_expiration);
        int16_t target_rpm_cached[NUM_THRUSTERS];
        for (int i = 0; i < NUM_THRUSTERS; i++) {
            target_rpm_cached[i] = target_req.rpm[i];
        }
        spin_unlock(target_req.lock, irq);

        // Handle any commands that have queued in the multicore fifo since the last tick
        // This updates the controller tunings at a deterministic point of the control loop
        bool param_modified = false;
        while (multicore_fifo_rvalid()) {
            param_modified = true;

            sio_fifo_req_t req = { .raw = multicore_fifo_pop_blocking() };
            if (req.type == CONTROLLER_PARAM_P_GAIN) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].Pgain = req.value;
                }
                controller_missing_params &= ~(1 << CONTROLLER_PARAM_P_GAIN);
            }
            else if (req.type == CONTROLLER_PARAM_I_GAIN) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].Igain = req.value;
                }
                controller_missing_params &= ~(1 << CONTROLLER_PARAM_I_GAIN);
            }
            else if (req.type == CONTROLLER_PARAM_I_BOUND) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].Ibound = req.value;
                }
                controller_missing_params &= ~(1 << CONTROLLER_PARAM_I_BOUND);
            }
            else if (req.type == CONTROLLER_PARAM_HARD_LIMIT) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].hardLimit = req.value;
                }
                controller_missing_params &= ~(1 << CONTROLLER_PARAM_HARD_LIMIT);
            }
            else if (req.type == CONTROLLER_PARAM_MIN_CMD) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].minCommand = req.value;
                }
                controller_missing_params &= ~(1 << CONTROLLER_PARAM_MIN_CMD);
            }
            else if (req.type == CONTROLLER_PARAM_INVERT_MASK) {
                for (int i = 0; i < NUM_THRUSTERS; i++) {
                    controller_state[i].inverted = (req.value & (1 << i) ? true : false);
                }
                controller_missing_params &= ~(1 << CONTROLLER_PARAM_INVERT_MASK);
            }
            else if (req.type == CONTROLLER_PARAM_RAW_MODE) {
                if (req.value) {
                    in_raw_mode = true;
                    safety_raise_fault(FAULT_RAW_MODE);
                }
                else {
                    in_raw_mode = false;
                    safety_lower_fault(FAULT_RAW_MODE);
                }
                controller_missing_params &= ~(1 << CONTROLLER_PARAM_RAW_MODE);
            }
        }

        if (param_modified) {
            // Zero controller after adjusting tunings
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                thruster_controller_zero(&controller_state[i]);
            }
        }

        // ========================================
        // Phase 2: Check Kill Conditions
        // ========================================

        uint8_t disabled_flags = 0;
        // Check for conditions which require the controller to be disabled for safety reasons
        //  - Kill switch is pulled
        //  - ESC Board does not have power (or it power cycled)
        //  - ESC startup delay (ESC board received power, but has yet to chime)
        //  - RPM command has gone stale
        //  - RPM timeout lockout (After RPM times out, don't accept any commands for a brief period to prevent jitter)
        //  - The controller has not yet been configured

        // Condition: Kill switch is pulled
        if (safety_kill_get_asserting_kill()) {
            disabled_flags |= (1 << CORE1_DISABLED_SAFETY_KILLED);
        }

        // Condition: ESC Board does not have power (or it power cycled)
        if (esc_board_was_off || !esc_board_on) {
            // If the ADC read callback saw that the ESC board turned off, or the board is currently off
            esc_board_was_off = false;
            disabled_flags |= (1 << CORE1_DISABLED_ESC_BOARD_OFF);

            // Set the startup delay. Once the ESC board, comes back on, this timeout will prevent commands while the
            // ESCs are still chiming.
            esc_startup_finished = make_timeout_time_ms(ESC_POWERUP_DELAY_MS);
        }

        // Condition: ESC startup delay (ESC board received power, but has yet to chime)
        // But only check if ESC board was off to not make disabled flags confusing
        else if (!time_reached(esc_startup_finished)) {
            disabled_flags |= (1 << CORE1_DISABLED_ESC_POWERON_DELAY);
        }

        // Condition: RPM command has gone stale
        if (target_rpm_expired) {
            disabled_flags |= (1 << CORE1_DISABLED_TGT_RPM_STALE);

            // If the thrusters were moving when the target RPM expired, then raise a fault, since the controller
            // timed out while commanding thrust. This is an issue with the ROS side, as it means it wasn't updating
            // us fast enough. Set the timeout penalty to prevent jitter
            if (dshot_thrusters_on) {
                safety_raise_fault(FAULT_RPM_CMD_TIMEOUT);
                command_timeout_penalty = make_timeout_time_ms(DSHOT_UPDATE_DISABLE_TIME_MS);
            }
        }

        // Condition: RPM timeout lockout (Don't accept any commands for a brief period after timeout to prevent jitter)
        if (!time_reached(command_timeout_penalty)) {
            disabled_flags |= (1 << CORE1_DISABLED_TGT_RPM_STALE_PENALTY);
        }

        // Condition: The controller has not yet been configured
        if (controller_missing_params) {
            disabled_flags |= (1 << CORE1_DISABLED_CONTROLLER_UNCONFIGURED);
        }

        // ========================================
        // Phase 3: Control Loop Overrides (Kill/Raw Mode)
        // ========================================

        // Kill controller if any disabled flag has been set
        if (disabled_flags != 0) {
            // Clear command and controller state. When the controller starts up again, it should be starting the same
            // as though it was a clean boot.
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                throttle_commands[i] = 0;
                thruster_controller_zero(&controller_state[i]);
            }
            last_tick = nil_time;
        }

        // If the controller is in raw mode, ignore the controller and send the throttle commands to it directly
        else if (in_raw_mode) {
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
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                // Force controller off if 0 rpm requested
                if (target_rpm_cached[i] == 0) {
                    throttle_commands[i] = 0;
                    thruster_controller_zero(&controller_state[i]);
                }
            }

            // If controller hasn't been disabled, compute tick difference
            // Compute the time delta between sending commands to the controller
            // This must be done as close as possible to dshot_update_thrusters
            absolute_time_t current_time = get_absolute_time();
            if (!is_nil_time(last_tick)) {
                // Only enable I gains if we have a proper time difference from the last control loop
                time_difference = absolute_time_diff_us(last_tick, current_time);
            }
            last_tick = current_time;
        }

        // ========================================
        // Phase 4: Dshot Command Transmit
        // ========================================

        // Send command, and compute min and max update timestamps
        // Note this command comes from the last controller tick (unless its been killed)
        dshot_update_thrusters(throttle_commands);
        // Time at which we are okay to send the next packet. Sending too fast will cause the ESC to drop the message
        absolute_time_t min_frame_time = make_timeout_time_us(DSHOT_MIN_FRAME_TIME_US);

        // Save last command to be stored under lock later
        int16_t last_command[NUM_THRUSTERS];
        for (int i = 0; i < NUM_THRUSTERS; i++) {
            last_command[i] = throttle_commands[i];
        }

        // ========================================
        // Phase 5: Dshot RPM Receive
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
                        // Store the resulting commthrottle_commands[i]and to be executed for next tick
                        int16_t new_cmd = thruster_controller_tick(&controller_state[i], target_rpm_cached[i], rpm[i],
                                                                   time_difference);

                        bool sign_changed =
                            (new_cmd > 0 && throttle_commands[i] < 0) || (new_cmd < 0 && throttle_commands[i] > 0);

                        if (!time_reached(controller_state[i].directionChangeTimeout)) {
                            thruster_controller_zero(&controller_state[i]);
                            new_cmd = 0;
                        }
                        else if (sign_changed) {
                            controller_state[i].directionChangeTimeout = make_timeout_time_ms(2);
                            thruster_controller_zero(&controller_state[i]);
                            new_cmd = 0;
                        }

                        throttle_commands[i] = new_cmd;
                    }
                    // Clear waiting, even on unsuccessful response
                    waiting &= ~(1 << i);
                    serviced = true;
                }
            }
        }

        // ========================================
        // Phase 6: Telemetry Writeback
        // ========================================

        irq = spin_lock_blocking(telem_state.lock);

        // Store the disabled flags
        telem_state.disabled_flags = disabled_flags;

        // Update total controller tick count
        if (telem_state.controller_tick_cnt < UINT16_MAX) {
            telem_state.controller_tick_cnt++;
        }

        for (int i = 0; i < NUM_THRUSTERS; i++) {
            // Store the command we just sent
            telem_state.thruster[i].cmd = last_command[i];

            // Writeback RPM data if valid
            if (rpm_valid[i]) {
                if (rpm[i] > INT16_MAX) {
                    telem_state.thruster[i].rpm = INT16_MAX;
                }
                else if (rpm[i] < INT16_MIN) {
                    telem_state.thruster[i].rpm = INT16_MIN;
                }
                else {
                    telem_state.thruster[i].rpm = (int16_t) rpm[i];
                }
                telem_state.thruster[i].rpm_missed_count = 0;
                telem_state.thruster[i].rpm_valid = true;

                // TODO: Handle an ESC being online, but a thruster is unplugged
                telem_state.thruster[i].thruster_ready = true;
            }
            // Keep track if RPM not valid
            else {
                if (telem_state.thruster[i].rpm_missed_count < TELEM_MAX_MISSED_DSHOT_PACKETS) {
                    // Increment missed count if we couldn't read it (and only in if statement to prevent overflows)
                    telem_state.thruster[i].rpm_missed_count++;

                    // Keep track of ticks where we lost a packet
                    if (telem_state.thruster[i].ticks_missed < UINT16_MAX) {
                        telem_state.thruster[i].ticks_missed++;
                    }
                }
                else {
                    // If we've missed too many, mark RPM as invalid
                    telem_state.thruster[i].rpm = 0;
                    telem_state.thruster[i].rpm_valid = false;
                    telem_state.thruster[i].thruster_ready = false;

                    // Keep track of ticks we took the thruster offline
                    if (telem_state.thruster[i].ticks_offline < UINT16_MAX) {
                        telem_state.thruster[i].ticks_offline++;
                    }

                    // Additionally, clear the last command
                    // We let the thruster miss a few telemetry packets and just run the an old command, but if we miss
                    // too many, we need to clear the command since the control loop hasn't been able refresh in a while
                    throttle_commands[i] = 0;
                    thruster_controller_zero(&controller_state[i]);
                }
            }
        }
        spin_unlock(telem_state.lock, irq);

        // ========================================
        // Phase 7: Dshot Min Delay Frame Wait
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

void core1_init(uint8_t board_id) {
    target_req.lock = spin_lock_init(spin_lock_claim_unused(true));
    telem_state.lock = spin_lock_init(spin_lock_claim_unused(true));
    safety_launch_core1(core1_main);

    // TODO: Move to parameters when ready

    // Inverted mask is in order index 0b3210 (highest thruster first)
    uint8_t inverted_mask;
    if (board_id == 0) {
        // Thrusters 4321
        inverted_mask = 0b1001;
    }
    else {
        // Thrusters 8765
        inverted_mask = 0b1001;
    }
    core1_set_thruster_inverted_mask(inverted_mask);

    core1_set_p_gain(100000);
    core1_set_i_gain(1000);
    core1_set_i_bound(300);
    core1_set_hard_limit(725);
    core1_set_min_command(0);
    core1_set_raw_mode(false);
}

void core1_update_target_rpm(const int16_t *rpm) {
    // Compute expiration outside of spin lock to avoid unnecessary work in critical section.
    absolute_time_t expiration = make_timeout_time_ms(DSHOT_MIN_UPDATE_RATE_MS);

    // Update shared variables under lock
    uint32_t irq = spin_lock_blocking(target_req.lock);
    target_req.rpm_expiration = expiration;
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        target_req.rpm[i] = rpm[i];
    }
    spin_unlock(target_req.lock, irq);
}

void core1_get_telem(struct core1_telem *telem_out, bool clear_telem) {
    // Compute time difference from last read, out of critical section to lower latency
    static absolute_time_t last_rate_read = { 0 };
    absolute_time_t now = get_absolute_time();
    int64_t time_diff_us64 = absolute_time_diff_us(last_rate_read, now);
    last_rate_read = now;

    // Limit to uint32_t
    if (time_diff_us64 > UINT32_MAX) {
        telem_out->time_delta_us = UINT32_MAX;
    }
    else if (time_diff_us64 < 0) {
        telem_out->time_delta_us = 0;
    }
    else {
        telem_out->time_delta_us = (uint32_t) time_diff_us64;
    }

    // Copy telemetry from shared memory under lock
    uint32_t irq = spin_lock_blocking(telem_state.lock);

    telem_out->controller_tick_cnt = telem_state.controller_tick_cnt;
    if (clear_telem) {
        telem_state.controller_tick_cnt = 0;
    }

    telem_out->disabled_flags = telem_state.disabled_flags;
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        telem_out->thruster[i].cmd = telem_state.thruster[i].cmd;
        telem_out->thruster[i].rpm = telem_state.thruster[i].rpm;
        telem_out->thruster[i].rpm_valid = telem_state.thruster[i].rpm_valid;
        telem_out->thruster[i].thruster_ready = telem_state.thruster[i].thruster_ready;
        telem_out->thruster[i].ticks_missed = telem_state.thruster[i].ticks_missed;
        telem_out->thruster[i].ticks_offline = telem_state.thruster[i].ticks_offline;

        if (clear_telem) {
            telem_state.thruster[i].ticks_missed = 0;
            telem_state.thruster[i].ticks_offline = 0;
        }
    }

    spin_unlock(telem_state.lock, irq);
}

// ========================================
// Core 0 Exported Parameter Control Funcs
// ========================================

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

bool core1_set_thruster_inverted_mask(int32_t value) {
    if (value < 0 || value >= (1 << NUM_THRUSTERS))
        return false;
    static_assert(NUM_THRUSTERS <= FIFO_REQ_VALUE_WIDTH, "Cannot fit all thrusters into fifo request");
    sio_fifo_req_t req = { .type = CONTROLLER_PARAM_INVERT_MASK, .value = value };
    multicore_fifo_push_blocking(req.raw);
    return true;
}

void core1_set_raw_mode(bool raw_mode) {
    sio_fifo_req_t req = { .type = CONTROLLER_PARAM_RAW_MODE, .value = (raw_mode ? 1 : 0) };
    multicore_fifo_push_blocking(req.raw);
}
