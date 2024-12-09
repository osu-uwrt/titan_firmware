#include "driver/dynamixel.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "titan/binary_info.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <stdio.h>

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint DYNAMIXEL_PIN = 4;

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl, GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i)
        ;

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl, GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

void on_dynamixel_error(dynamixel_error_t error) {
    printf("Dynamixel Driver Error: %d (arg: %d) - %s line %d\n", error.fields.error, error.fields.wrapped_error_code,
           (error.fields.error_source == DYNAMIXEL_SOURCE_COMMS ? "dynamixel_comms" : "dynamixel_schedule"),
           error.fields.line);
    return;
}

void on_dynamixel_event(enum dynamixel_event event, dynamixel_id id) {
    switch (event) {
    case DYNAMIXEL_EVENT_DISCONNECTED:
        printf("Servo %d Disconnected\n", id);
        break;
    case DYNAMIXEL_EVENT_CONNECTED: {
        struct dynamixel_eeprom *eeprom = dynamixel_get_eeprom(id);

        printf("Servo Connected!\n");
        printf("--- Servo %d EEPROM ---\n", id);
        printf("  Model Number: %d\n", eeprom->model_num);
        printf("  Model Info: %ld\n", eeprom->model_info);
        printf("  Firmware Version: %d\n", eeprom->firmware_version);
        printf("  ID: %d\n", eeprom->id);
        printf("  Baud Rate: %d\n", eeprom->baud_rate);
        printf("  Return Delay Time: %d\n", eeprom->return_delay_time);
        printf("  Drive Mode: %d\n", eeprom->drive_mode);
        printf("  Operating Mode: %d\n", eeprom->operating_mode);
        printf("  Secondary ID: %d\n", eeprom->secondary_id);
        printf("  Protocol: %d\n", eeprom->protocol_type);
        printf("  Homing Offset: %ld\n", eeprom->homing_offset);
        printf("  Temperature Limit: %d\n", eeprom->temperature_limit);
        printf("  Min Voltage Limit: %d\n", eeprom->min_voltage_limit);
        printf("  Max Voltage Limit: %d\n", eeprom->max_voltage_limit);
        printf("  PWM Limit: %d\n", eeprom->pwm_limit);
        printf("  Velocity Limit: %ld\n", eeprom->velocity_limit);
        printf("  Min Position Limit: %ld\n", eeprom->min_position_limit);
        printf("  Max Position Limit: %ld\n", eeprom->max_position_limit);
        printf("  Startup Config: %d\n", eeprom->startup_config);
        printf("-------------------\n");

        dynamixel_enable_torque(2, true);

        break;
    }
    case DYNAMIXEL_EVENT_RAM_READ: {
        volatile struct dynamixel_ram *ram = dynamixel_get_ram(id);

        printf("--- Servo %d RAM ---\n", id);
        printf("  Torque Enable: %d\n", ram->torque_enable);
        printf("  Moving: %d\n", ram->moving);
        printf("  Realtime Tick: %d\n", ram->realtime_tick);
        printf("  Hardware Error Status: %d\n", ram->hardware_error_status);
        printf("  Goal Position: %ld\n", ram->goal_position);
        printf("  Present Position: %ld\n", ram->present_position);
        printf("  Present Velocity: %ld\n", ram->present_velocity);
        printf("  Present Voltage: %d.%d V\n", ram->present_input_voltage / 10, ram->present_input_voltage % 10);
        printf("  Present Temperature: %d C\n", ram->present_temperature);
        printf("-------------------\n");
        break;
    }
    case DYNAMIXEL_EVENT_ALERT:
        printf("Servo %d Hardware Error Status Set!\n", id);
        break;

    default:
        printf("Receieved unknown dynamixel event %d for ID %d\n", event, id);
        break;
    }
}

int main() {
    watchdog_enable(5000, true);
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    LOG_INFO("%s", FULL_BUILD_TAG);

    sleep_ms(2000);
    LOG_INFO("Starting up dynamixel");
    uint8_t servos[] = { 2 };
    dynamixel_init(pio0, 0, DYNAMIXEL_PIN, servos, 1, on_dynamixel_error, on_dynamixel_event);

    sleep_ms(100);

    bool value = true;

    while (true) {
        watchdog_update();

        gpio_put(LED_PIN, value);
        value = !value;

        sleep_ms(250);

        if (get_bootsel_button()) {
            dynamixel_set_target_position(2, 500);
        }
        else {
            dynamixel_set_target_position(2, 2048);
        }
    }
    return 0;
}
