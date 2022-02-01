#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/exception.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/psm.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include <rmw_microros/rmw_microros.h>

#include "drivers/safety.h"
#include "hw/dio.h"
#include "hw/dshot.h"
#include "hw/esc_pwm.h"

// ========================================
// Fault Management Functions
// ========================================

volatile uint32_t *fault_list = &watchdog_hw->scratch[6];

void safety_raise_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((*fault_list & (1u<<fault_id)) == 0) {
        printf("Fault %d Raised\n", fault_id);

        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled during
        // the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = save_and_disable_interrupts();
        
        *fault_list |= (1<<fault_id);
        dio_set_fault_led(true);
        
        restore_interrupts(prev_interrupt_state);
    }
}

void safety_lower_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((*fault_list & (1u<<fault_id)) != 0) {
        printf("Fault %d Lowered\n", fault_id);
        
        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled during
        // the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = save_and_disable_interrupts();
        
        *fault_list &= ~(1u<<fault_id);
        dio_set_fault_led((*fault_list) != 0);
        
        restore_interrupts(prev_interrupt_state);
    }
}



// ========================================
// Kill Switch Management Functions
// ========================================

static absolute_time_t last_kill_switch_change;
static bool last_state_asserting_kill = true; // Start asserting kill
struct kill_switch_state kill_switch_states[riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES] = 
    {[0 ... riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES-1] = { .enabled = false }};

/**
 * @brief Contains all callback functions to be called when robot is killed
 * Called from any function that will kill the robot
 * 
 * This function should only be called when safety is initialized
 */
static void safety_kill_robot(void) {
    last_state_asserting_kill = true;
    last_kill_switch_change = get_absolute_time();

    // Note: Any calls made in this function must be safe to be called from interrupts
    // This is because safety_kill_switch_update can be called from interrupts

    #if HW_USE_DSHOT
    dshot_stop_thrusters();
    #endif
    #if HW_USE_PWM
    esc_pwm_stop_thrusters();
    #endif
}

/**
 * @brief Refreshes kill switches to check for any timeouts
 * It is responsible for re-enabling the robot after all of the kill switches have been released.
 * 
 * This function should only be called when safety is initialized
 */
static void safety_refresh_kill_switches(void) {
    absolute_time_t now = get_absolute_time();

    // Check all kill switches for asserting kill
    bool asserting_kill = false;
    int num_switches_enabled = 0;
    for (int i = 0; i < riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES; i++) {
        if (kill_switch_states[i].enabled) {
            num_switches_enabled++;

            // Kill if asserting kill or if timeout expired when requiring update
            if (kill_switch_states[i].asserting_kill || 
                    (kill_switch_states[i].needs_update && absolute_time_diff_us(now, kill_switch_states[i].update_timeout) < 0)) {
                asserting_kill = true;
                break;
            }
        }
    }

    // If no kill switches are enabled, force into kill as a precaution
    if (num_switches_enabled == 0) {
        asserting_kill = true;
    }

    // Update last state, and notify of kill if needed
    if (last_state_asserting_kill != asserting_kill) {
        last_state_asserting_kill = asserting_kill;

        if (asserting_kill) {
            safety_kill_robot();
        } else {
            last_kill_switch_change = get_absolute_time();
        }
    }
}

void safety_kill_switch_update(uint8_t switch_num, bool asserting_kill, bool needs_update){
    valid_params_if(SAFETY, switch_num < riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES);

    kill_switch_states[switch_num].asserting_kill = asserting_kill;
    kill_switch_states[switch_num].update_timeout = make_timeout_time_ms(KILL_SWITCH_TIMEOUT_MS);
    kill_switch_states[switch_num].needs_update = needs_update;
    kill_switch_states[switch_num].enabled = true;

    if (safety_initialized && asserting_kill) {
        safety_kill_robot();
    }
}

void safety_kill_msg_process(const riptide_msgs2__msg__KillSwitchReport *msg) {
    if (!rmw_uros_epoch_synchronized()){
        printf("Safety Kill Switch: No Time Synchronization for Comand Verification!\n");
        safety_raise_fault(FAULT_ROS_SOFT_FAIL);
        return;
    }

    // Check to make sure message isn't old
    int64_t command_time = (msg->header.stamp.sec * 1000) + 
                            (msg->header.stamp.nanosec / 1000000);
    int64_t command_time_diff = rmw_uros_epoch_millis() - command_time;

    if (command_time_diff > SOFTWARE_KILL_MAX_TIME_DIFF_MS || command_time_diff < -SOFTWARE_KILL_MAX_TIME_DIFF_MS) {
        printf("Stale kill switch report received: %d ms old\n", command_time_diff);
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    // Make sure kill switch id is valid
    if (msg->kill_switch_id >= riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES || 
            msg->kill_switch_id == riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_PHYSICAL) {
        printf("Invalid kill switch id used %d\n", msg->kill_switch_id);
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    // Make sure frame id isn't too large
    if (msg->header.frame_id.size >= SOFTWARE_KILL_FRAME_STR_SIZE) {
        printf("Software Kill Frame ID too large\n");
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    struct kill_switch_state* kill_entry = &kill_switch_states[msg->kill_switch_id];

    if (kill_entry->enabled && kill_entry->asserting_kill && !msg->switch_asserting_kill && 
            strncmp(kill_entry->locking_frame, msg->header.frame_id.data, SOFTWARE_KILL_FRAME_STR_SIZE)) {
        printf("Invalid frame ID to unlock kill switch %d ('%s' expected, '%s' requested)\n", msg->kill_switch_id, kill_entry->locking_frame, msg->header.frame_id.data);
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    // Set frame id of the switch requesting disable
    // This can technically override previous kills and allow one node to kill when another is stopping
    // However, this is mainly to prevent getting locked out if, for example, rqt crashed and the robot was killed
    // If multiple points are needed to be separate, separate kill switch IDs should be used
    // This will protect from someone unexpectedly unkilling the robot by publishing a non assert kill
    // since it must be asserted as killed by the node to take ownership of the lock
    if (msg->switch_asserting_kill) {
        strncpy(kill_entry->locking_frame, msg->header.frame_id.data, msg->header.frame_id.size);
        kill_entry->locking_frame[msg->header.frame_id.size] = '\0';
    }

    safety_kill_switch_update(msg->kill_switch_id, msg->switch_asserting_kill, msg->switch_needs_update);
}

bool safety_kill_get_asserting_kill(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_initialized);
    return last_state_asserting_kill;
}

absolute_time_t safety_kill_get_last_change(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_initialized);
    return last_kill_switch_change;
}



// ========================================
// Watchdog Crash Reporting Functions
// ========================================

// Use of Watchdog Scratch Registers
// Can really only be considered valid if 
// scratch[0]: Last Crash Action
//  - UNKNOWN_SAFETY_PREINIT: Unknown, crashed after safety_setup
//  - UNKNOWN_SAFETY_ACTIVE: Unknown, crashed after safety_init
//  - PANIC: Set on panic function call
//     scratch[1]: Address of panic string (Won't be dereferenced for safety)
//  - HARD_FAULT: Set in the hard fault exception handler (Any unhandled exception raises this)
//     scratch[1]: Faulting Address
//  - ASSERT_FAIL: Set in assertion callback
//     scratch[1]: Faulting File String Address
//     scratch[2]: Faulting File Line
//  - IN_ROS_TRANSPORT_LOOP: Set while blocking for response from ROS agent
// scratch[3]: Watchdog Reset Counter (LSB order)
//     Byte 0: Total Watchdog Reset Counter
//     Byte 1: Panic Reset Counter
//     Byte 2: Hard Fault Reset Counter
//     Byte 3: Assertion Fail Reset Counter
// scratch[6]: Bitwise Fault List
// scratch[7]: Depth Sensor Backup Data
//     Default: Should be set to 0xFFFFFFFF on clean boot
//     Will be set during zeroing of the depth sensor

#define UNKNOWN_SAFETY_PREINIT  0x1035001
#define UNKNOWN_SAFETY_ACTIVE   0x1035002
#define PANIC                   0x1035003
#define HARD_FAULT              0x1035004
#define ASSERT_FAIL             0x1035005
#define IN_ROS_TRANSPORT_LOOP   0x1035006

// Very useful for debugging, prevents cpu from resetting while execution is halted for debugging
// However, this should be ideally be disabled when not debugging in the event something goes horribly wrong
#define PAUSE_WATCHDOG_ON_DEBUG 1

static volatile uint32_t *reset_reason_reg = &watchdog_hw->scratch[0];
static volatile uint32_t *reset_counter = &watchdog_hw->scratch[3];

// Defined in hard_fault_handler.S
extern void safety_hard_fault_handler(void);
static exception_handler_t original_hardfault_handler = NULL;

// Assertion Handling
extern void __real___assert_func(const char *file, int line, const char *func, const char *failedexpr);

void __wrap___assert_func(const char *file, int line, const char *func, const char *failedexpr) {
    *reset_reason_reg = ASSERT_FAIL;
    watchdog_hw->scratch[1] = (uint32_t) file;
    watchdog_hw->scratch[2] = (uint32_t) line;
    if ((*reset_counter & 0xFF000000) != 0xFF000000) {
        *reset_counter += 0x1000000;
    }

    // Remove the hard fault exception handler so it doesn't overwrite panic data when the breakpoint is hit
    if (original_hardfault_handler != NULL) {
        exception_restore_handler(HARDFAULT_EXCEPTION, original_hardfault_handler);
    }
    
    __real___assert_func(file, line, func, failedexpr);
}

/**
 * @brief Custom panic handler to save the panic in watchdog scratch registers
 * 
 * SHOULD NOT BE CALLED DIRECTLY. Use `panic` instead
 * 
 * @param fmt printf like format string for panic
 * @param ... Args for format string
 */
void safety_panic(const char *fmt, ...) {
    *reset_reason_reg = PANIC;
    watchdog_hw->scratch[1] = (uint32_t) fmt;
    if ((*reset_counter & 0xFF00) != 0xFF00) {
        *reset_counter += 0x100;
    }

    puts("\n*** PANIC ***\n");

    if (fmt) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        puts("\n");
    }

    // Remove the hard fault exception handler so it doesn't overwrite panic data when the breakpoint is hit
    if (original_hardfault_handler != NULL) {
        exception_restore_handler(HARDFAULT_EXCEPTION, original_hardfault_handler);
    }

    __breakpoint();
}

static bool had_watchdog_reboot = false;
static uint32_t last_reset_reason = 0;
static uint32_t last_fault_list = 0;
static uint32_t prev_scratch1 = 0;
static uint32_t prev_scratch2 = 0;

static void safety_print_last_reset_cause(void){
    if (had_watchdog_reboot) {
        printf("Watchdog Reset (Total Crashes: %d", (*reset_counter) & 0xFF);
        if ((*reset_counter) & 0xFF00) {
            printf(" - Panics: %d", ((*reset_counter) >> 8) & 0xFF);
        }
        if ((*reset_counter) & 0xFF0000) {
            printf(" - Hard Faults: %d", ((*reset_counter) >> 16) & 0xFF);
        }
        if ((*reset_counter) & 0xFF000000) {
            printf(" - Assert Fails: %d", ((*reset_counter) >> 24) & 0xFF);
        }

        if (last_fault_list != 0) {
            printf(") (Faults: 0x%x", last_fault_list);
        }

        printf(") - Reason: ");

        if (last_reset_reason == UNKNOWN_SAFETY_PREINIT) {
            printf("UNKNOWN_SAFETY_PREINIT\n");
        } else if (last_reset_reason == UNKNOWN_SAFETY_ACTIVE) {
            printf("UNKNOWN_SAFETY_ACTIVE\n");
        } else if (last_reset_reason == PANIC) {
            printf("PANIC (Message: 0x%08x)\n", prev_scratch1);
        } else if (last_reset_reason == HARD_FAULT) {
            printf("HARD_FAULT (Fault Address: 0x%08x)\n", prev_scratch1);
        } else if (last_reset_reason == ASSERT_FAIL) {
            printf("ASSERT_FAIL (File: 0x%08x Line: %d)\n", prev_scratch1, prev_scratch2);
        } else if (last_reset_reason == IN_ROS_TRANSPORT_LOOP) {
            printf("ROS Agent Lost\n");
        } else {
            printf("Invalid Data in Reason Register\n");
        }
    } else {
        printf("Clean boot\n");
    }
}

/**
 * @brief Does processing of the last reset cause and prints it to serial.
 * Also increments the total watchdog reset counter
 * 
 * Should only be called once and before reset reason is overwritten
 */
static void safety_process_last_reset_cause(void) {
    // Handle data in watchdog registers
    bool should_raise_fault = false;
    if (watchdog_enable_caused_reboot()) {
        last_reset_reason = *reset_reason_reg;
        last_fault_list = *fault_list;
        prev_scratch1 = watchdog_hw->scratch[1];
        prev_scratch2 = watchdog_hw->scratch[2];
        had_watchdog_reboot = true;
        
        if (last_reset_reason != IN_ROS_TRANSPORT_LOOP) {
            if (((*reset_counter) & 0xFF) != 0xFF) 
                *reset_counter = *reset_counter + 1;
            should_raise_fault = true;
        }

        if (*reset_counter != 0) {
            should_raise_fault = true;
        }

        // Clear any previous faults
        *fault_list = 0;
    } else {
        *reset_counter = 0;
        *fault_list = 0;
        watchdog_hw->scratch[7] = 0xFFFFFFFF;
        had_watchdog_reboot = false;
    }

    safety_print_last_reset_cause();
    if (should_raise_fault) {
        safety_raise_fault(FAULT_WATCHDOG_RESET);
    }
}

// ========================================
// Safety Limetime Functions
// ========================================

bool safety_initialized = false;
bool safety_is_setup = false;

void safety_setup(void) {
    hard_assert_if(LIFETIME_CHECK, safety_is_setup || safety_initialized);

    // Set hardfault handler
    original_hardfault_handler = exception_set_exclusive_handler(HARDFAULT_EXCEPTION, &safety_hard_fault_handler);

    safety_process_last_reset_cause();

    // Set reset reason
    *reset_reason_reg = UNKNOWN_SAFETY_PREINIT;
    safety_is_setup = true;

    // Enable slow watchdog while connecting
    watchdog_enable(3000, PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_init(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_is_setup || safety_initialized);

    safety_initialized = true;
    *reset_reason_reg = UNKNOWN_SAFETY_ACTIVE;

    // Populate the last kill switch change time to when safety is set up
    last_kill_switch_change = get_absolute_time();

    // Set tight watchdog timer for normal operation
    watchdog_enable(250, PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_tick(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_is_setup);

    // Check for any kill switch timeouts
    if (safety_initialized) {
        safety_refresh_kill_switches();
    }

    watchdog_update();
}