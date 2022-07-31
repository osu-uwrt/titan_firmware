#include <stdbool.h>
#include <stdint.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"

#include "basic_logger/logging.h"

#include "drivers/safety.h"
#include "actuators/torpedo.h"
#include "torpedo.pio.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "torpedo"

#define TORPEDO_DISCHARGED_THRESHOLD 20.25
#define TORPEDO_CHARGED_THRESHOLD 21.75
#define TORPEDO_VDIV_CAL (160.5022)

const uint CHARGED_LED_PIN = BUILTIN_LED3_PIN;

static const uint arm_pin = TORP_ARM_PIN;
bi_decl(bi_1pin_with_name(TORP_ARM_PIN, "Torpedo Arm"));

#define NUM_COILS 3
// PIO requires that coil pins be consecutive for side-set
#define FIRST_COIL_PIN COIL_1_PIN
static_assert(COIL_1_PIN+1 == COIL_2_PIN, "Coil pins must be consecutive");
static_assert(COIL_2_PIN+1 == COIL_3_PIN, "Coil pins must be consecutive");
bi_decl(bi_1pin_with_name(COIL_1_PIN, "Torpedo Coil 1"));
bi_decl(bi_1pin_with_name(COIL_2_PIN, "Torpedo Coil 2"));
bi_decl(bi_1pin_with_name(COIL_3_PIN, "Torpedo Coil 3"));

static const uint torpedo_select_pins[] = {TORP_SEL_1_PIN, TORP_SEL_2_PIN};
bi_decl(bi_1pin_with_name(TORP_SEL_1_PIN, "Torpedo Select 1"));
bi_decl(bi_1pin_with_name(TORP_SEL_2_PIN, "Torpedo Select 2"));

#define NUM_TORPEDOS (sizeof(torpedo_select_pins)/sizeof(*torpedo_select_pins))

#define ARM_LEVEL_ARMED    1
#define ARM_LEVEL_DISARMED 0
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
struct torpedo_data {
    uint16_t timings[ACTUATOR_NUM_TORPEDO_TIMINGS];
    bool fired;
} torpedo_data[NUM_TORPEDOS] = {0};

/**
 * @brief The active torpedo num currently using the timer.
 * Should be 0 if no torpedo is actively running
 *
 * This is the sole source of information for which torpedo is firing
 */
uint active_torpedo_num = 0;


bool torpedo_initialized = false;

/**
 * @brief Holds allocation of state machine
 */
static const PIO torpedo_pio = pio0;
static uint torpedo_pio_offset;
static uint torpedo_pio_sm;

static void torpedo_fired_callback(void);
static void torpedo_adc_cb(void);

// ========================================
// Initialization Routines
// ========================================

void torpedo_initialize(void) {
    hard_assert_if(LIFETIME_CHECK, torpedo_initialized);

    // Charged led
    gpio_init(CHARGED_LED_PIN);
    gpio_set_dir(CHARGED_LED_PIN, GPIO_OUT);

    // Torpedo pins
    gpio_init(arm_pin);
    gpio_put(arm_pin, TORP_SEL_LEVEL_OFF);
    gpio_set_dir(arm_pin, true);

    for (uint i = 0; i < NUM_TORPEDOS; i++) {
        uint pin = torpedo_select_pins[i];
        gpio_init(pin);
        gpio_put(pin, TORP_SEL_LEVEL_OFF);
        gpio_set_dir(pin, true);
    }

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

    torpedo_initialized = true;
}


// ========================================
// Timing Management
// ========================================

// Subtract the time it takes to pull data before running loop
#define ON_SUBTRACT     2
#define OFF_SUBTRACT    4

#define IS_ON_TIMING_TYPE(timing_type) (((timing_type) % 2) == 0)
static_assert(IS_ON_TIMING_TYPE(ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME), "Unexpected timing calculation");

enum actuator_command_result torpedo_set_timings(struct torpedo_timing_cmd *timings) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    // Input command checking
    if (timings->torpedo_num < 1 || timings->torpedo_num > NUM_TORPEDOS) {
        return ACTUATOR_RESULT_FAILED;
    }

    if (timings->timing_type >= ACTUATOR_NUM_TORPEDO_TIMINGS) {
        return ACTUATOR_RESULT_FAILED;
    }

    uint compensation_value;

    if (IS_ON_TIMING_TYPE(timings->timing_type)) {
        compensation_value = ON_SUBTRACT;
    } else {
        compensation_value = OFF_SUBTRACT;
    }


    if (timings->time_us < compensation_value){
        return ACTUATOR_RESULT_FAILED;
    }

    // Don't allow timings to change while torpedo is firing
    if (active_torpedo_num == timings->torpedo_num) {
        return ACTUATOR_RESULT_FAILED;
    }

    // Set the timings
    LOG_INFO("Setting Torpedo Timings for torpedo %d (Timing type: %d, timing: %d us)", timings->torpedo_num, timings->timing_type, timings->time_us);

    torpedo_data[timings->torpedo_num-1].timings[timings->timing_type] = timings->time_us - compensation_value;
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
// ADC Management
// ========================================

static float torpedo_charge = 0;
static bool torpedo_charged = false;

static bool torpedo_check_charged(void) {
    if (torpedo_charged){
        torpedo_charged = torpedo_charge >= TORPEDO_DISCHARGED_THRESHOLD;
    } else {
        torpedo_charged = torpedo_charge >= TORPEDO_CHARGED_THRESHOLD;
    }
    return torpedo_charged;
}

static void torpedo_adc_cb(void) {
    torpedo_charge = (float)adc_fifo_get() / TORPEDO_VDIV_CAL;
    gpio_put(CHARGED_LED_PIN, torpedo_check_charged());
}

// ========================================
// Movement Management
// ========================================

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

void torpedo_fired_callback(void) {
    torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset);
    gpio_put(torpedo_select_pins[active_torpedo_num-1], TORP_SEL_LEVEL_OFF);
    torpedo_data[active_torpedo_num-1].fired = true;
    active_torpedo_num = 0;
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
    active_torpedo_num = torpedo_num;

    // Set initial timer for torpedo
    torpedo_fire_sequence(torpedo_pio, torpedo_pio_sm, this_torpedo->timings);

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
        // If trpedo is active mark as fired
        torpedo_data[active_torpedo-1].fired = true;
        active_torpedo_num = 0;
    }

    // Clear pio state machine
    torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset);

    // Clear all GPIO pins
    uint32_t torpedo_sel_mask = 0;
    for (uint i = 0; i < NUM_TORPEDOS; i++){
        torpedo_sel_mask |= (1<<torpedo_select_pins[i]);
    }
    gpio_put_masked(torpedo_sel_mask, TORP_SEL_LEVEL_OFF);

    gpio_put(arm_pin, ARM_LEVEL_DISARMED);

}