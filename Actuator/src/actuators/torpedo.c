#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"

#include "basic_logger/logging.h"

#include "drivers/safety.h"
#include "actuators/torpedo.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "torpedo"

// TODO: Find out charge time
#define CHARGE_TIME_MS 3000

static const uint torpedo_hardware_alarm_num = 0;

static const uint arm_pin = TORP_ARM_PIN;
bi_decl(bi_1pin_with_name(TORP_ARM_PIN, "Torpedo Arm"));

static const uint coil_pins[] = {COIL_1_PIN, COIL_2_PIN, COIL_3_PIN};
bi_decl(bi_1pin_with_name(COIL_1_PIN, "Torpedo Coil 1"));
bi_decl(bi_1pin_with_name(COIL_2_PIN, "Torpedo Coil 2"));
bi_decl(bi_1pin_with_name(COIL_3_PIN, "Torpedo Coil 3"));

static const uint torpedo_select_pins[] = {TORP_SEL_1_PIN, TORP_SEL_2_PIN};
bi_decl(bi_1pin_with_name(TORP_SEL_1_PIN, "Torpedo Select 1"));
bi_decl(bi_1pin_with_name(TORP_SEL_2_PIN, "Torpedo Select 2"));

#define NUM_TORPEDOS (sizeof(torpedo_select_pins)/sizeof(*torpedo_select_pins))
#define NUM_COILS    (sizeof(coil_pins)/sizeof(*coil_pins))

#define ARM_LEVEL_ARMED    1
#define ARM_LEVEL_DISARMED 0
#define COIL_LEVEL_ON  1
#define COIL_LEVEL_OFF 0
#define TORP_SEL_LEVEL_ON  1
#define TORP_SEL_LEVEL_OFF 0

// Small sanity check to make sure that the coils defined above equal the coils defined in the timings command
static_assert((ACTUATOR_NUM_TORPEDO_TIMINGS+1) == NUM_COILS*2);

/**
 * @brief Data containing instance data for each torpedo
 */
struct torpedo_data {
    uint16_t timings[ACTUATOR_NUM_TORPEDO_TIMINGS];
    bool fired;
    uint coil_num;
    uint coil_active;
} torpedo_data[NUM_TORPEDOS] = {0};

/**
 * @brief The active torpedo num currently using the timer.
 * Should be 0 if no torpedo is actively running
 * 
 * This is the sole source of information for which torpedo is firing
 */
uint active_torpedo_num = 0;


bool torpedo_initialized = false;

static void torpedo_alarm_callback(uint alarm_num);

void torpedo_initialize(void) {
    hard_assert_if(LIFETIME_CHECK, torpedo_initialized);

    hardware_alarm_claim(torpedo_hardware_alarm_num);
    hardware_alarm_cancel(torpedo_hardware_alarm_num);
    hardware_alarm_set_callback(torpedo_hardware_alarm_num, torpedo_alarm_callback);    
    irq_set_priority(TIMER_IRQ_0 + torpedo_hardware_alarm_num, 0);  // Set to highest priority interrupt

    gpio_init(arm_pin);
    gpio_put(arm_pin, TORP_SEL_LEVEL_OFF);
    gpio_set_dir(arm_pin, true);

    for (uint i = 0; i < NUM_TORPEDOS; i++) {
        uint pin = torpedo_select_pins[i];
        gpio_init(pin);
        gpio_put(pin, TORP_SEL_LEVEL_OFF);
        gpio_set_dir(pin, true);
    }

    for (uint i = 0; i < NUM_COILS; i++) {
        uint pin = coil_pins[i];
        gpio_init(pin);
        gpio_put(pin, TORP_SEL_LEVEL_OFF);
        gpio_set_dir(pin, true);
    }

    torpedo_initialized = true;
}


// ========================================
// Timing Management
// ========================================

enum actuator_command_result torpedo_set_timings(struct torpedo_timing_cmd *timings) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    // Input command checking
    if (timings->torpedo_num < 1 || timings->torpedo_num > NUM_TORPEDOS) {
        return ACTUATOR_RESULT_FAILED;
    }

    if (timings->timing_type >= ACTUATOR_NUM_TORPEDO_TIMINGS) {
        return ACTUATOR_RESULT_FAILED;
    }

    if (timings->time_us == 0){
        return ACTUATOR_RESULT_FAILED;
    }

    // Don't allow timings to change while torpedo is firing
    if (active_torpedo_num == timings->torpedo_num) {
        return ACTUATOR_RESULT_FAILED;
    }

    // Set the timings
    LOG_INFO("Setting Torpedo Timings for torpedo %d (Timing type: %d, timing: %d us)", timings->torpedo_num, timings->timing_type, timings->time_us);

    torpedo_data[timings->torpedo_num-1].timings[timings->timing_type] = timings->time_us;
    return ACTUATOR_RESULT_SUCCESSFUL;
}

void torpedo_populate_missing_timings(struct missing_timings_status* missing_timings) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    missing_timings->torpedo1_coil1_on_timing = (torpedo_data[0].timings[ACTUATOR_TORPEDO_TIMING_COIL1_ON_TIME] == 0);
    missing_timings->torpedo1_coil1_2_delay_timing = (torpedo_data[0].timings[ACTUATOR_TORPEDO_TIMING_COIL1_2_DELAY_TIME] == 0);
    missing_timings->torpedo1_coil2_on_timing = (torpedo_data[0].timings[ACTUATOR_TORPEDO_TIMING_COIL2_ON_TIME] == 0);
    missing_timings->torpedo1_coil2_3_delay_timing = (torpedo_data[0].timings[ACTUATOR_TORPEDO_TIMING_COIL2_3_DELAY_TIME] == 0);
    missing_timings->torpedo1_coil3_on_timing = (torpedo_data[0].timings[ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME] == 0);

    missing_timings->torpedo2_coil1_on_timing = (torpedo_data[1].timings[ACTUATOR_TORPEDO_TIMING_COIL1_ON_TIME] == 0);
    missing_timings->torpedo2_coil1_2_delay_timing = (torpedo_data[1].timings[ACTUATOR_TORPEDO_TIMING_COIL1_2_DELAY_TIME] == 0);
    missing_timings->torpedo2_coil2_on_timing = (torpedo_data[1].timings[ACTUATOR_TORPEDO_TIMING_COIL2_ON_TIME] == 0);
    missing_timings->torpedo2_coil2_3_delay_timing = (torpedo_data[1].timings[ACTUATOR_TORPEDO_TIMING_COIL2_3_DELAY_TIME] == 0);
    missing_timings->torpedo2_coil3_on_timing = (torpedo_data[1].timings[ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME] == 0);
}


// ========================================
// Movement Management
// ========================================

absolute_time_t charged_time;
static bool torpedo_check_charged(void) {
    return absolute_time_diff_us(charged_time, get_absolute_time()) > 0;
}

static void torpedo_fire_next_sequence(struct torpedo_data *this_torpedo) {
    // Generate timings
    uint16_t timing = this_torpedo->timings[COIL_ACTIVITY_TO_TIMING(this_torpedo->coil_num, this_torpedo->coil_active)];
    hardware_alarm_set_target(torpedo_hardware_alarm_num, delayed_by_us(get_absolute_time(), timing));

    // Turn on coil if needed in the sequence
    if (this_torpedo->coil_active) {
        gpio_put(coil_pins[this_torpedo->coil_num], COIL_LEVEL_ON);
    }
}

/**
 * @brief Alarm for wwhen the current torpedo sequence finishes
 */
static void torpedo_alarm_callback(__unused uint alarm_num) {
    uint torpedo_num = active_torpedo_num;
    hard_assert(torpedo_num >= 1 && torpedo_num <= NUM_TORPEDOS);

    struct torpedo_data *this_torpedo = &torpedo_data[torpedo_num-1];

    // Turn off coil if needed in the sequence
    if (this_torpedo->coil_active) {
        gpio_put(coil_pins[this_torpedo->coil_num], COIL_LEVEL_OFF);
    }

    // If the current fired coil is the last coil in the sequence, end right then
    if (this_torpedo->coil_num+1 == NUM_COILS) {
        gpio_put(torpedo_select_pins[torpedo_num-1], TORP_SEL_LEVEL_OFF);
        this_torpedo->fired = true;
        active_torpedo_num = 0;
    } else {
        // Go to next step of firing sequence
        // Coil 1 Active -> Coil 1 inactive -> Coil 2 Active -> Coil 2 Inactive -> Coil 3
        if (!this_torpedo->coil_active) {
            this_torpedo->coil_num++;
        }
        this_torpedo->coil_active = !this_torpedo->coil_active;

        torpedo_fire_next_sequence(this_torpedo);
    }
}


// ===== Public Functions =====

enum torpedo_state torpedo_get_state(uint8_t torpedo_id) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    valid_params_if(TORPEDO, torpedo_id >= 1 && torpedo_id <= NUM_TORPEDOS);
    struct torpedo_data *this_torpedo = &torpedo_data[torpedo_id-1];

    // Ensure all of the timings are initialized (non-zero)
    for (int i = 0; i < ACTUATOR_NUM_TORPEDO_TIMINGS; i++) {
        if (this_torpedo->timings[i] == 0) {
            return TORPEDO_STATE_UNINITIALIZED;
        }
    }

    // Handle global disarm
    if (gpio_get_out_level(arm_pin) == ARM_LEVEL_DISARMED) {
        return TORPEDO_STATE_DISARMED;
    }

    // Handle individual torpedo state
    if (active_torpedo_num == torpedo_id) {
        return TORPEDO_STATE_FIRING;
    } else if (this_torpedo->fired) {
        return TORPEDO_STATE_FIRED;
    } else if (torpedo_check_charged()) {
        return TORPEDO_STATE_READY;
    } else {
        return TORPEDO_STATE_CHARGING;
    }
}

enum actuator_command_result torpedo_fire(struct fire_torpedo_cmd *cmd) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    // Ensure input command is valid
    uint8_t torpedo_num = cmd->torpedo_num;
    if (torpedo_num <= 0 || torpedo_num > NUM_TORPEDOS) {
        return ACTUATOR_RESULT_FAILED;
    }

    // Don't allow firing of torpedos when killed
    if (safety_kill_get_asserting_kill()) {
        return ACTUATOR_RESULT_FAILED;
    }

    // If another alarm has reserved the torpedo alarm (meaning its firing), don't fire
    if (active_torpedo_num != 0 && active_torpedo_num != torpedo_num) {
        return ACTUATOR_RESULT_FAILED;
    }

    // Handle the various states. Can only fire when ready but other states might return values other than failure
    switch (torpedo_get_state(torpedo_num)) {
        case TORPEDO_STATE_FIRED:
            return ACTUATOR_RESULT_SUCCESSFUL;
        case TORPEDO_STATE_FIRING:
            return ACTUATOR_RESULT_RUNNING;
        case TORPEDO_STATE_READY:
            break;
        default:
            return ACTUATOR_RESULT_FAILED;
    }

    LOG_INFO("Firing Torpedo %d", torpedo_num);


    // Set initial torpedo state
    gpio_put(torpedo_select_pins[torpedo_num-1], TORP_SEL_LEVEL_ON);

    struct torpedo_data *this_torpedo = &torpedo_data[torpedo_num-1];
    this_torpedo->coil_num = 0;
    this_torpedo->coil_active = true;
    active_torpedo_num = torpedo_num;

    // Run the first sequence for the torpedo
    torpedo_fire_next_sequence(this_torpedo);

    return ACTUATOR_RESULT_RUNNING;
}

enum actuator_command_result torpedo_arm(void) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    // Don't allow arming if killed
    if (safety_kill_get_asserting_kill()) {
        return ACTUATOR_RESULT_FAILED;
    }

    // Don't need to re-arm if already armed
    if (gpio_get_out_level(arm_pin) == ARM_LEVEL_ARMED) {
        return ACTUATOR_RESULT_SUCCESSFUL;
    }

    LOG_INFO("Arming Torpedos");

    // The torpedos should be reset to not-fired when disarmed and re-armed
    for (uint i = 0; i < NUM_TORPEDOS; i++) {
        torpedo_data[i].fired = false;
    }

    charged_time = make_timeout_time_ms(CHARGE_TIME_MS);
    gpio_put(arm_pin, ARM_LEVEL_ARMED);

    return ACTUATOR_RESULT_SUCCESSFUL;
}

enum actuator_command_result torpedo_disarm(void) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    // Don't allow disarming if a torpedo is actively firing
    if (active_torpedo_num != 0) {
        return ACTUATOR_RESULT_FAILED;
    }

    // Don't need to disarm if already disarmed
    if (gpio_get_out_level(arm_pin) == ARM_LEVEL_DISARMED) {
        return ACTUATOR_RESULT_SUCCESSFUL;
    }

    LOG_INFO("Disarming Torpedos");

    gpio_put(arm_pin, ARM_LEVEL_DISARMED);

    return ACTUATOR_RESULT_SUCCESSFUL;
}

void torpedo_safety_disable(void) {
    // It should be fine that this be called even before initialization
    // since this_torpedo->firing should never be true before init

    uint active_torpedo = active_torpedo_num;
    if (active_torpedo != 0) {
        // Cancel and release alarm
        hardware_alarm_cancel(torpedo_hardware_alarm_num);
        torpedo_data[active_torpedo-1].fired = true;
        active_torpedo_num = 0;
    }

    // Clear all GPIO pins

    // Doing this convoluted way to get the mask since the compiler can theoretically optimize it this way
    // And if not doing the mask it'll still loop anways, so might as well try to get it optimized
    uint32_t coil_mask = 0;
    for (uint i = 0; i < NUM_COILS; i++) {
        coil_mask |= (1<<coil_pins[i]);
    }
    gpio_put_masked(coil_mask, COIL_LEVEL_OFF);


    uint32_t torpedo_sel_mask = 0;
    for (uint i = 0; i < NUM_TORPEDOS; i++){
        torpedo_sel_mask |= (1<<torpedo_select_pins[i]);
    }
    gpio_put_masked(torpedo_sel_mask, TORP_SEL_LEVEL_OFF);


    gpio_put(arm_pin, ARM_LEVEL_DISARMED);
    
}