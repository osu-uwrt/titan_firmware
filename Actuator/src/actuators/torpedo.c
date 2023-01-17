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
#include "actuators/arm_state.h"
#include "torpedo.pio.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "torpedo"

#define TORPEDO_DISCHARGED_THRESHOLD 20.25
#define TORPEDO_CHARGED_THRESHOLD 21.75
#define TORPEDO_VDIV_CAL (160.5022)

const uint CHARGED_LED_PIN = BUILTIN_LED3_PIN;

static const uint discharge_pin = TORP_DRAIN_PIN;
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
#define DISCHARGE_LEVEL_CHARGED 0
#define DISCHARGE_LEVEL_DISCHARGED 1

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
    bool firing;
    uint16_t timings[ACTUATOR_NUM_TORPEDO_TIMINGS];
} torpedo_data[NUM_TORPEDOS] = {0};

bool torpedo_initialized = false;

/**
 * @brief Holds allocation of state machine
 */
static const PIO torpedo_pio = pio0;
static uint torpedo_pio_offset;
static uint torpedo_pio_sm;

static enum armed_state torpedo_armed_state;

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
    torpedo_armed_state = ARMED_STATE_DISARMED;

    for (uint i = 0; i < NUM_TORPEDOS; i++) {
        uint pin = torpedo_select_pins[i];
        gpio_init(pin);
        gpio_put(pin, TORP_SEL_LEVEL_OFF);
        gpio_set_dir(pin, true);

        torpedo_data[i].firing = false;
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

bool torpedo_set_timings(uint8_t torpedo_num, enum torpedo_timing_type timing_type, uint16_t time_us) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

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

    // Don't allow timings to change while torpedo is firing
    if (torpedo_armed_state == ARMED_STATE_ARMED) {
        return false;
    }

    // Set the timings
    LOG_INFO("Setting Torpedo Timings for torpedo %d (Timing type: %d, timing: %d us)", torpedo_num, timing_type, time_us);

    torpedo_data[torpedo_num-1].timings[timing_type] = time_us - compensation_value;
    return true;
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

    // Handle individual torpedo state
    if(torpedo_armed_state == ARMED_STATE_DISARMED) {
        return TORPEDO_STATE_DISARMED;
    } else if (torpedo_data[torpedo_id-1].firing) {
        return TORPEDO_STATE_FIRING;
    } else if (torpedo_check_charged()) {
        return TORPEDO_STATE_READY;
    } else {
        return TORPEDO_STATE_CHARGING;
    }
}

void torpedo_fired_callback(void) {
    torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset, FIRST_COIL_PIN);

    for(uint i = 0; i < NUM_TORPEDOS; i++) {
        if(torpedo_data[i].firing) {
            gpio_put(torpedo_select_pins[i], TORP_SEL_LEVEL_OFF);
            torpedo_data[i].firing = false;
        }
    }

    torpedo_disarm();
}

bool torpedo_fire(uint8_t torpedo_num) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    if (torpedo_num <= 0 || torpedo_num > NUM_TORPEDOS) {
        return false;
    }

    // Don't allow firing of torpedos when killed
    if (safety_kill_get_asserting_kill()) {
        return false;
    }

    if (torpedo_armed_state != ARMED_STATE_ARMED || torpedo_get_state(torpedo_num) != TORPEDO_STATE_READY) {
        return false;
    }

    LOG_INFO("Firing Torpedo %d", torpedo_num);
    torpedo_armed_state = ARMED_STATE_FIRING;

    // Set initial torpedo state
    gpio_put(torpedo_select_pins[torpedo_num-1], TORP_SEL_LEVEL_ON);

    struct torpedo_data *this_torpedo = &torpedo_data[torpedo_num-1];
    this_torpedo->firing = true;
    // Set initial timer for torpedo
    torpedo_fire_sequence(torpedo_pio, torpedo_pio_sm, this_torpedo->timings);

    return true;
}

bool torpedo_arm(void) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    // Don't allow arming if killed
    if (safety_kill_get_asserting_kill()) {
        return false;
    }

    if(torpedo_armed_state != ARM_LEVEL_DISARMED) {
        return false;
    }

    LOG_INFO("Arming Torpedos");
    torpedo_armed_state = ARMED_STATE_ARMED;

    gpio_put(arm_pin, ARM_LEVEL_ARMED);

    return false;
}

bool torpedo_disarm(void) {
    hard_assert_if(LIFETIME_CHECK, !torpedo_initialized);

    if(torpedo_armed_state != ARMED_STATE_ARMED) {
        return false;
    }

    LOG_INFO("Disarming Torpedos");
    torpedo_armed_state = ARMED_STATE_DISARMED;

    gpio_put(arm_pin, ARM_LEVEL_DISARMED);

    return true;
}

void torpedo_safety_disable(void) {
    // It should be fine that this be called even before initialization
    // since this_torpedo->firing should never be true before init

    for(uint i = 0; i < NUM_TORPEDOS; i++) {
        torpedo_data[i].firing = false;
    }
    torpedo_armed_state = ARMED_STATE_DISARMED;

    // Clear pio state machine
    torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset, FIRST_COIL_PIN);

    // Clear all GPIO pins
    uint32_t torpedo_sel_mask = 0;
    for (uint i = 0; i < NUM_TORPEDOS; i++){
        torpedo_sel_mask |= (1<<torpedo_select_pins[i]);
    }
    gpio_put_masked(torpedo_sel_mask, TORP_SEL_LEVEL_OFF);

    gpio_put(arm_pin, ARM_LEVEL_DISARMED);

    torpedo_discharge();
}

void torpedo_discharge() { 
    gpio_put(discharge_pin, DISCHARGE_LEVEL_DISCHARGED);
}

enum armed_state torpedo_get_armed_state(void) {
    return torpedo_armed_state;
}