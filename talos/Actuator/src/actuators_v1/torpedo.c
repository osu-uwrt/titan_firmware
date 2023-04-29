#include <stdbool.h>
#include <stdint.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"
#include "pico/sync.h"

#include "basic_logger/logging.h"

#include "safety_interface.h"
#include "actuators.h"
#include "torpedo.pio.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "torpedo"

#define TORPEDO_DISCHARGED_THRESHOLD 20.25
#define TORPEDO_CHARGED_THRESHOLD 21.75
#define TORPEDO_VDIV_CAL (160.5022)

bi_decl(bi_1pin_with_name(TORP_ARM_PIN, "Torpedo Arm"));
bi_decl(bi_1pin_with_name(TORP_DRAIN_PIN, "Torpedo Drain"));

#define NUM_COILS 3
// PIO requires that coil pins be reverse consecutive for side-set
// See actuator mk2 board pin definitions and torpedo pio file for more details
#define FIRST_COIL_PIN COIL_3_PIN   // This must be the coil pin with the lowest pin number
static_assert(COIL_3_PIN+1 == COIL_2_PIN, "Coil pins must be consecutive");
static_assert(COIL_2_PIN+1 == COIL_1_PIN, "Coil pins must be consecutive");
bi_decl(bi_1pin_with_name(COIL_1_PIN, "Torpedo Coil 1"));
bi_decl(bi_1pin_with_name(COIL_2_PIN, "Torpedo Coil 2"));
bi_decl(bi_1pin_with_name(COIL_3_PIN, "Torpedo Coil 3"));

static const uint torpedo_select_pins[] = {TORP_SEL_1_PIN, TORP_SEL_2_PIN};
bi_decl(bi_1pin_with_name(TORP_SEL_1_PIN, "Torpedo Select 1"));
bi_decl(bi_1pin_with_name(TORP_SEL_2_PIN, "Torpedo Select 2"));

#define NUM_TORPEDOS (sizeof(torpedo_select_pins)/sizeof(*torpedo_select_pins))

#define ARM_LEVEL_ARMED    0
#define ARM_LEVEL_DISARMED 1
#define DRAIN_LEVEL_NODRAIN 0
#define DRAIN_LEVEL_DRAIN 1

#define COIL_LEVEL_ON  1
#define COIL_LEVEL_OFF 0
#define TORP_SEL_LEVEL_ON  1
#define TORP_SEL_LEVEL_OFF 0

// Sanity check to make sure that the coils defined above equal the coils defined in the timings command
static_assert((ACTUATOR_NUM_TORPEDO_TIMINGS+1) == NUM_COILS*2, "Timings does not match number of coils");
// Sanity check to make sure torpedo timings in the protocol matches what pio expects
static_assert(ACTUATOR_NUM_TORPEDO_TIMINGS == torpedo_num_timings, "Protocol number of timings does not match pio state machine");

/**
 * @brief Data containing instance data for each torpedo
 */
static uint16_t torpedo_timings[NUM_TORPEDOS][ACTUATOR_NUM_TORPEDO_TIMINGS] = {0};
static bool torpedo_timings_valid = false;

/**
 * @brief The index of the next torpedo to fire
 */
static volatile uint next_torpedo_index = 0;

/**
 * @brief True if a torpedo is actively firing
 */
static volatile bool torpedo_firing = false;

/**
 * @brief Useful variable holding a mask of all select pins to allow for easy clearing of the select lines
 */
static uint32_t torpedo_sel_mask = 0;
static uint32_t torpedo_sel_off_mask = 0;

/**
 * @brief Holds allocation of state machine
 */
static const PIO torpedo_pio = pio0;
static uint torpedo_pio_offset;
static uint torpedo_pio_sm;

static void torpedo_fired_callback(void);
static void torpedo_adc_cb(void);

// ========================================
// Internal Functions
// ========================================

void torpedo_initialize(void) {
    hard_assert_if(ACTUATORS, actuators_initialized);

    // Torpedo pins
    gpio_init(TORP_ARM_PIN);
    gpio_put(TORP_ARM_PIN, ARM_LEVEL_DISARMED);
    gpio_set_dir(TORP_ARM_PIN, true);

    gpio_init(TORP_DRAIN_PIN);
    gpio_put(TORP_DRAIN_PIN, DRAIN_LEVEL_DRAIN);
    gpio_set_dir(TORP_DRAIN_PIN, true);

    // Compute select mask and set the pins
    torpedo_sel_mask = 0;
    for (uint i = 0; i < NUM_TORPEDOS; i++){
        torpedo_sel_mask |= 1 << torpedo_select_pins[i];
        torpedo_sel_off_mask |= TORP_SEL_LEVEL_OFF << torpedo_select_pins[i];
    }
    gpio_init_mask(torpedo_sel_mask);
    gpio_put_masked(torpedo_sel_mask, torpedo_sel_off_mask);
    gpio_set_dir_out_masked(torpedo_sel_mask);

    // VDIV processing
    adc_init();
    adc_gpio_init(TORP_CHARGE_LVL);
    adc_select_input(TORP_CHARGE_LVL - 26);
    adc_set_clkdiv(48000.f);
    adc_fifo_setup(true, true, 1, false, false);
    adc_irq_set_enabled(true);
    adc_run(true);
    irq_set_exclusive_handler(ADC_IRQ_FIFO, torpedo_adc_cb);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    torpedo_pio_offset = pio_add_program(torpedo_pio, &torpedo_program);
    torpedo_pio_sm = pio_claim_unused_sm(torpedo_pio, true);
    torpedo_program_init(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset, FIRST_COIL_PIN, torpedo_fired_callback);
}

bool torpedo_arm(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    // Enter critical section to make sure a kill doesn't fire in the middle of arming
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Verify a kill interrupt didn't fire before getting to this code
    if (!actuators_armed) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Killed";
        return false;
    }

    // Clear the firing state (as if we re-arm, we probably want to start from the beginning)
    // TODO: Make sure this is the behavior we want
    next_torpedo_index = 0;

    // Arm and allow charging
    gpio_put(TORP_DRAIN_PIN, DRAIN_LEVEL_NODRAIN);
    gpio_put(TORP_ARM_PIN, ARM_LEVEL_ARMED);
    restore_interrupts(prev_interrupts);

    LOG_INFO("Arming Torpedos");

    return true;
}

void torpedo_safety_disable(void) {
    // It should be fine to do nothing before initialization since the
    // pins haven't even been configured to do anything before initialized is true
    if (!actuators_initialized){
        return;
    }

    // Critical section to avoid getting our disable interrupted
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Clear all select GPIO pins always, as this should stop any firing in progress
    gpio_put_masked(torpedo_sel_mask, torpedo_sel_off_mask);

    // Disarm and discharge the torpedos
    gpio_put(TORP_ARM_PIN, ARM_LEVEL_DISARMED);
    gpio_put(TORP_DRAIN_PIN, DRAIN_LEVEL_DRAIN);

    // Only reset the PIO state machine if the torpedo is actively firing
    // This is due to the time torpedo_reset takes, so we'll call this outside of the critical section
    if (torpedo_firing) {
        torpedo_firing = false;
        restore_interrupts(prev_interrupts);

        // Clear pio state machine
        torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset, FIRST_COIL_PIN);

        // Report that the actuation was aborted
        // This is important since we responed over ROS that the actuator fired, but now we're cancelling it
        // Due to how brief these pulses are, it is worth the occasional fault when we pull the kill switch over thinking
        // the actuator fired and some random kill event stopped it silently (like a timeout)
        safety_raise_fault(FAULT_ACTUATOR_FAILURE);
    }
    else {
        restore_interrupts(prev_interrupts);
    }

}


// ========================================
// Timing Management
// ========================================

// Subtract the time it takes to pull data before running loop
#define ON_SUBTRACT     2
#define OFF_SUBTRACT    4

#define IS_ON_TIMING_TYPE(timing_type) (((timing_type) % 2) == 0)
static_assert(IS_ON_TIMING_TYPE(ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME), "Unexpected timing calculation");

bool torpedo_set_timings(uint8_t torpedo_num, enum torpedo_timing_type timing_type, uint16_t time_us) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    // Input command checking
    if (torpedo_num < 1 || torpedo_num > NUM_TORPEDOS) {
        return false;
    }

    if (timing_type >= ACTUATOR_NUM_TORPEDO_TIMINGS) {
        return false;
    }

    uint compensation_value;

    if (IS_ON_TIMING_TYPE(timing_type)) {
        compensation_value = ON_SUBTRACT;
    } else {
        compensation_value = OFF_SUBTRACT;
    }


    if (time_us < compensation_value){
        return false;
    }

    // Don't allow timings to change while torpedo is armed
    if (actuators_armed) {
        return false;
    }

    // Set the timings
    LOG_INFO("Setting Torpedo Timings for torpedo %d (Timing type: %d, timing: %d us)", torpedo_num, timing_type, time_us);

    torpedo_timings[torpedo_num-1][timing_type] = time_us - compensation_value;

    // Refresh boolean if all timings are configured
    bool timings_valid = true;
    for (uint torpedo_index = 0; torpedo_index < NUM_TORPEDOS; torpedo_index++) {
        for (uint i = 0; i < ACTUATOR_NUM_TORPEDO_TIMINGS; i++) {
            if (torpedo_timings[torpedo_index][i] == 0) {
                timings_valid = false;
            }
        }
    }
    torpedo_timings_valid = timings_valid;

    return true;
}

// ========================================
// ADC Management
// ========================================

static float torpedo_charge = 0;
static bool torpedo_check_charged(void) {
    static bool torpedo_charged = false;

    if (torpedo_charged){
        torpedo_charged = torpedo_charge >= TORPEDO_DISCHARGED_THRESHOLD;
    } else {
        torpedo_charged = torpedo_charge >= TORPEDO_CHARGED_THRESHOLD;
    }
    return torpedo_charged;
}

static void torpedo_adc_cb(void) {
    torpedo_charge = (float)adc_fifo_get() / TORPEDO_VDIV_CAL;
}

// ========================================
// Movement Management
// ========================================

// ===== Public Functions =====

enum torpedo_state torpedo_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    // Handle individual torpedo state
    if (!torpedo_timings_valid) {
        return TORPEDO_STATE_UNINITIALIZED;
    } else if(!actuators_armed) {
        return TORPEDO_STATE_DISARMED;
    } else if (torpedo_firing) {
        return TORPEDO_STATE_FIRING;
    } else if (torpedo_check_charged()) {
        return TORPEDO_STATE_READY;
    } else {
        return TORPEDO_STATE_CHARGING;
    }
    // TODO: Add error on charge timeout
}

void torpedo_fired_callback(void) {
    // Critical section to make sure the cleanup isn't interrupts
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Clear the select pins always, just to be safe
    gpio_put_masked(torpedo_sel_mask, torpedo_sel_off_mask);

    // Only reset torpedos/increment index if we're firing
    if (torpedo_firing) {
        torpedo_firing = false;
        next_torpedo_index++;
        restore_interrupts(prev_interrupts);

        // torpedo_reset actually takes a bit of time, so we'll use torpedo_firing to gate calls to this
        torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset, FIRST_COIL_PIN);
    }
    else {
        restore_interrupts(prev_interrupts);
    }

    // TODO: Check for expected discharge of capacitor
}

bool torpedo_fire(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    // Don't allow firing of torpedos when killed
    if (safety_kill_get_asserting_kill()) {
        *errMsgOut = "Kill Switch Removed";
        return false;
    }

    // Make sure that actuators are armed
    if (!actuators_armed) {
        *errMsgOut = "Not Armed";
        return false;
    }

    // Make sure the next index to fire exists
    if (next_torpedo_index >= NUM_TORPEDOS) {
        *errMsgOut = "All Torpedos Fired";
        return false;
    }

    // Make sure we're not already firing a torpedo
    if (torpedo_firing) {
        *errMsgOut = "Already Firing";
        return false;
    }

    // Make sure we're charged
    if (!torpedo_check_charged()) {
        *errMsgOut = "Not Charged";
        return false;
    }

    // It should be fine to save this outside of a critical section, as this is only incremented by the firing callback
    // As we already checked that we aren't firing, its fine to just take this as it is
    uint torpedo_index = next_torpedo_index;

    // Ensure all of the timings are initialized (non-zero)
    for (int i = 0; i < ACTUATOR_NUM_TORPEDO_TIMINGS; i++) {
        if (torpedo_timings[torpedo_index][i] == 0) {
            *errMsgOut = "Missing Timings";
            return false;
        }
    }

    // Begin the PIO firing sequence
    // (in critical section so an untimely kill won't throw us into an undefined state)
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Make sure that the volatile fields didn't change during other checks while we're in the critical section
    if (safety_kill_get_asserting_kill() || !actuators_armed) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Killed";
        return false;
    }

    torpedo_firing = true;
    gpio_put(torpedo_select_pins[torpedo_index], TORP_SEL_LEVEL_ON);
    if (!torpedo_fire_sequence(torpedo_pio, torpedo_pio_sm, torpedo_timings[torpedo_index])) {
        // If the fire failed, clean up and throw an error
        torpedo_firing = false;
        gpio_put(torpedo_select_pins[torpedo_index], TORP_SEL_LEVEL_OFF);
        restore_interrupts(prev_interrupts);

        // Reset the PIO SM in hope that it'll fix next time
        torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset, FIRST_COIL_PIN);

        safety_raise_fault(FAULT_ACTUATOR_FAILURE);
        *errMsgOut = "PIO Error";
        return false;
    }

    restore_interrupts(prev_interrupts);

    LOG_INFO("Firing Torpedo %d", torpedo_index + 1);

    return true;
}

bool torpedo_notify_reload(const char **errMsgOut) {
    if (torpedo_firing) {
        *errMsgOut = "Firing in progress";
        return false;
    }

    next_torpedo_index = 0;
    return true;
}
