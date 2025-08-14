#include "solenoid.h"

#include "safety_interface.h"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "titan/logger.h"

#define SOLENOID_OPEN 1
#define SOLENOID_CLOSED 0

#define KILL_PRESSURE_OPEN_TIME_MS 1000
#define KILL_WATER_OPEN_TIME_MS 3000
#define KILL_SOLENOID_SPACING_TIME_MS 100
#define KILL_NUM_CYCLES 5

const int solenoid_pins[SOLENOID_COUNT] = { SOLENOID0_PIN, SOLENOID1_PIN, SOLENOID2_PIN, PUMP_PIN };

bool solenoid_states[SOLENOID_COUNT] = { false, false, false };

// Kill behavior
bool kill_requested = false;
bool kill_active = false;
uint num_cycles = 0;
absolute_time_t next_pressure_time;
absolute_time_t next_water_time;
absolute_time_t next_wait_time;

bool cycle_requested = false;
bool cycle_active = false;
uint cycle_idx = 0;
uint cycle_pressure_ms = 0;

void solenoid_request_cycle_ignoring_kill(int32_t ms);

void solenoid_init() {
    for (int i = 0; i < SOLENOID_COUNT; i++) {
        int pin = solenoid_pins[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, SOLENOID_CLOSED);
    }

    next_pressure_time = at_the_end_of_time;
    next_water_time = at_the_end_of_time;
    next_wait_time = at_the_end_of_time;
}

static void solenoid_set_ignoring_kill(int number, bool open) {
    int idx = number - 1;
    if (idx >= 0 && number <= SOLENOID_COUNT) {
        gpio_put(solenoid_pins[idx], open ? SOLENOID_OPEN : SOLENOID_CLOSED);
        solenoid_states[idx] = open;
    }
    else {
        LOG_ERROR("cannot set solenoid: invalid number %d", number);
    }
}

void solenoid_set(int number, bool open) {
    if (kill_active || kill_requested || cycle_active || cycle_requested)
        return;

    solenoid_set_ignoring_kill(number, open);
}

bool solenoid_get(int number) {
    return solenoid_states[number - 1];
}

void solenoid_start_kill_routine() {
    if (kill_active)
        return;

    kill_requested = true;

    cycle_active = false;
    cycle_requested = false;

    cycle_pressure_ms = KILL_PRESSURE_OPEN_TIME_MS;
}

static void solenoid_tick_kill() {
    if (kill_requested && !kill_active) {
        num_cycles = 0;

        // Close everything before we start
        for (int i = 0; i < SOLENOID_COUNT; i++) {
            solenoid_set_ignoring_kill(i + 1, false);
        }

        // next_water_time = at_the_end_of_time;
        // next_pressure_time = at_the_end_of_time;

        // next_wait_time = get_absolute_time();

        kill_active = true;
        kill_requested = false;
    }

    if (num_cycles >= KILL_NUM_CYCLES) {
        kill_active = false;
    }

    if (kill_active && !cycle_active && !cycle_requested) {
        solenoid_request_cycle_ignoring_kill(KILL_PRESSURE_OPEN_TIME_MS);
        num_cycles++;
    }

    // if (kill_requested && !kill_active) {
    //     num_cycles = 0;

    //     // Close everything before we start
    //     for (int i = 0; i < SOLENOID_COUNT; i++) {
    //         solenoid_set_ignoring_kill(i + 1, false);
    //     }

    //     next_water_time = at_the_end_of_time;
    //     next_pressure_time = at_the_end_of_time;

    //     next_wait_time = get_absolute_time();

    //     kill_active = true;
    //     kill_requested = false;
    // }

    // if (time_reached(next_wait_time)) {
    //     for (int i = 0; i < SOLENOID_COUNT; i++) {
    //         solenoid_set_ignoring_kill(i + 1, false);
    //     }

    //     if (num_cycles >= 2 * KILL_NUM_CYCLES) {
    //         next_pressure_time = at_the_end_of_time;
    //         next_water_time = at_the_end_of_time;
    //         next_wait_time = at_the_end_of_time;
    //         kill_active = false;
    //     }
    //     else if (num_cycles % 2 == 0) {
    //         next_pressure_time = make_timeout_time_ms(KILL_SOLENOID_SPACING_TIME_MS);
    //         next_water_time = at_the_end_of_time;
    //     }
    //     else {
    //         next_water_time = make_timeout_time_ms(KILL_SOLENOID_SPACING_TIME_MS);
    //         next_pressure_time = at_the_end_of_time;
    //     }

    //     next_wait_time = at_the_end_of_time;
    // }

    // if (time_reached(next_pressure_time)) {
    //     kill_active ? solenoid_set_ignoring_kill(PRESSURE_SOLENOID_NUM + 1, true) :
    //                   solenoid_set(PRESSURE_SOLENOID_NUM + 1, true);
    //     next_wait_time = make_timeout_time_ms(KILL_PRESSURE_OPEN_TIME_MS);
    //     num_cycles++;

    //     next_pressure_time = at_the_end_of_time;
    // }

    // if (time_reached(next_water_time)) {
    //     kill_active ? solenoid_set_ignoring_kill(WATER_SOLENOID_NUM + 1, true) :
    //                   solenoid_set(WATER_SOLENOID_NUM + 1, true);
    //     next_wait_time = make_timeout_time_ms(KILL_WATER_OPEN_TIME_MS);
    //     num_cycles++;

    //     next_water_time = at_the_end_of_time;
    // }
}

void solenoid_request_cycle_ignoring_kill(int32_t ms) {
    if (cycle_active || cycle_requested)
        return;

    cycle_pressure_ms = ms;
    cycle_requested = true;
}

void solenoid_request_cycle(int32_t ms) {
    // Reject incoming requests if killing
    if (safety_kill_get_asserting_kill())
        return;

    solenoid_request_cycle_ignoring_kill(ms);
}

static void solenoid_tick_cycle() {
    if (cycle_requested && !cycle_active) {
        cycle_idx = 0;

        // Close everything before we start
        for (int i = 0; i < SOLENOID_COUNT; i++) {
            solenoid_set_ignoring_kill(i + 1, false);
        }

        next_water_time = at_the_end_of_time;
        next_pressure_time = at_the_end_of_time;

        next_wait_time = get_absolute_time();

        cycle_active = true;
        cycle_requested = false;
    }

    if (time_reached(next_wait_time)) {
        for (int i = 0; i < SOLENOID_COUNT; i++) {
            solenoid_set_ignoring_kill(i + 1, false);
        }

        // if (num_cycles >= 2 * KILL_NUM_CYCLES) {
        //     next_pressure_time = at_the_end_of_time;
        //     next_water_time = at_the_end_of_time;
        //     next_wait_time = at_the_end_of_time;
        //     kill_active = false;
        // }
        if (cycle_idx == 0) {
            next_pressure_time = make_timeout_time_ms(KILL_SOLENOID_SPACING_TIME_MS);
            next_water_time = at_the_end_of_time;
        }
        else if (cycle_idx == 1) {
            next_water_time = make_timeout_time_ms(KILL_SOLENOID_SPACING_TIME_MS);
            next_pressure_time = at_the_end_of_time;
        }
        else {
            cycle_active = false;
        }

        next_wait_time = at_the_end_of_time;
    }

    if (time_reached(next_pressure_time)) {
        solenoid_set_ignoring_kill(PRESSURE_SOLENOID_NUM + 1, true);
        next_wait_time = make_timeout_time_ms(cycle_pressure_ms);
        cycle_idx++;

        next_pressure_time = at_the_end_of_time;
    }

    if (time_reached(next_water_time)) {
        solenoid_set_ignoring_kill(WATER_SOLENOID_NUM + 1, true);
        next_wait_time = make_timeout_time_ms(KILL_WATER_OPEN_TIME_MS);
        cycle_idx++;

        next_water_time = at_the_end_of_time;
    }

    // if (kill_requested && !kill_active) {
    //     num_cycles = 0;

    //     // Close everything before we start
    //     for (int i = 0; i < SOLENOID_COUNT; i++) {
    //         solenoid_set_ignoring_kill(i + 1, false);
    //     }

    //     next_water_time = at_the_end_of_time;
    //     next_pressure_time = at_the_end_of_time;

    //     next_wait_time = get_absolute_time();

    //     kill_active = true;
    //     kill_requested = false;
    // }

    // if (time_reached(next_wait_time)) {
    //     for (int i = 0; i < SOLENOID_COUNT; i++) {
    //         solenoid_set_ignoring_kill(i + 1, false);
    //     }

    //     if (num_cycles >= 2 * KILL_NUM_CYCLES) {
    //         next_pressure_time = at_the_end_of_time;
    //         next_water_time = at_the_end_of_time;
    //         next_wait_time = at_the_end_of_time;
    //         kill_active = false;
    //     }
    //     else if (num_cycles % 2 == 0) {
    //         next_pressure_time = make_timeout_time_ms(KILL_SOLENOID_SPACING_TIME_MS);
    //         next_water_time = at_the_end_of_time;
    //     }
    //     else {
    //         next_water_time = make_timeout_time_ms(KILL_SOLENOID_SPACING_TIME_MS);
    //         next_pressure_time = at_the_end_of_time;
    //     }

    //     next_wait_time = at_the_end_of_time;
    // }

    // if (time_reached(next_pressure_time)) {
    //     solenoid_set_ignoring_kill(PRESSURE_SOLENOID_NUM + 1, true);
    //     next_wait_time = make_timeout_time_ms(KILL_PRESSURE_OPEN_TIME_MS);
    //     num_cycles++;

    //     next_pressure_time = at_the_end_of_time;
    // }

    // if (time_reached(next_water_time)) {
    //     solenoid_set_ignoring_kill(WATER_SOLENOID_NUM + 1, true);
    //     next_wait_time = make_timeout_time_ms(KILL_WATER_OPEN_TIME_MS);
    //     num_cycles++;

    //     next_water_time = at_the_end_of_time;
    // }
}

void solenoid_tick() {
    solenoid_tick_kill();
    solenoid_tick_cycle();
}
