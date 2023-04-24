#include "pico/stdlib.h"

#include "basic_logger/logging.h"
#include "build_version.h"

#include "ros.h"
#include "safety_interface.h"
#include "led.h"
#include "hardware/structs/ioqspi.h"
#include "pico/sync.h"

#include "dynamixel/dynamixel.h"
#include "dynamixel/async_uart.h"

#ifdef MICRO_ROS_TRANSPORT_USB
#include "micro_ros_pico/transport_usb.h"
#endif

#ifdef MICRO_ROS_TRANSPORT_CAN
#include "can_mcp251Xfd/canbus.h"
#include "micro_ros_pico/transport_can.h"
#endif

#define is_receiver 0

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

#define UROS_CONNECT_PING_TIME_MS 1000
#define HEARTBEAT_TIME_MS 100
#define FIRMWARE_STATUS_TIME_MS 1000
#define LED_UPTIME_INTERVAL_MS 250

// Initialize all to nil time
// For background timers, they will fire immediately
// For ros timers, they will be reset before being ticked by start_ros_timers
absolute_time_t next_heartbeat = {0};
absolute_time_t next_status_update = {0};
absolute_time_t next_led_update = {0};
absolute_time_t next_connect_ping = {0};

uint8_t data_buf[8] = "Hello 0";
uint8_t recv_buf[8];

void my_write_cb(enum async_uart_tx_err error);

void my_read_cb(enum async_uart_rx_err error, uint8_t *data, size_t len) {
    if (!error) {
        if (is_receiver) {
            async_uart_write(data, len, false, my_write_cb);
        } else {
            printf("Received: \"%.8s\"\n", data);
        }
    } else {
        printf("ERR: %d\n", error);
        if (is_receiver && error != ASYNC_UART_RX_BUSY) {
            async_uart_read(recv_buf, sizeof(recv_buf), my_read_cb);
        }
    }
}

void __time_critical_func(my_write_cb)(enum async_uart_tx_err error) {
    if (!error) {
        async_uart_read(recv_buf, sizeof(recv_buf), my_read_cb);
    }
    else {
        printf("TX Busy?\n");
    }
}

int64_t transmit_pacer(__unused alarm_id_t id, __unused void *user_data) {
    data_buf[6]++;
    if (data_buf[6] > '9') {
        data_buf[6] = '0';
    }
    printf("Transmit: \"%.8s\"\n", data_buf);
    async_uart_write(data_buf, sizeof(data_buf), false, my_write_cb);
    return 500 * 1000;
}

/**
 * @brief Check if a timer is ready. If so advance it to the next interval.
 *
 * This will also raise a fault if timers are missed
 *
 * @param next_fire_ptr A pointer to the absolute_time_t holding the time the timer should next fire
 * @param interval_ms The interval the timer fires at
 * @return true The timer has fired, any action which was waiting for this timer should occur
 * @return false The timer has not fired
 */
static inline bool timer_ready(absolute_time_t *next_fire_ptr, uint32_t interval_ms, bool error_on_miss) {
    absolute_time_t time_tmp = *next_fire_ptr;
    if (time_reached(time_tmp)) {
        time_tmp = delayed_by_ms(time_tmp, interval_ms);
        if (time_reached(time_tmp)) {
            unsigned int i = 0; \
            while(time_reached(time_tmp)) {
                time_tmp = delayed_by_ms(time_tmp, interval_ms);
                i++;
            }
            LOG_WARN("Missed %u runs of %s timer 0x%p", i, (error_on_miss ? "critical" : "non-critical"), next_fire_ptr);
            if (error_on_miss)
                safety_raise_fault(FAULT_TIMER_MISSED);
        }
        *next_fire_ptr = time_tmp;
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief Starts all ROS timers after ROS starts up
 *
 * Ensures that ROS timers will be staggered from ROS bringup and won't flood the queue all at once
 * This also prevents the timer code from warning about missing runs (as ROS hasn't been running)
 */
static void start_ros_timers() {
    next_heartbeat = make_timeout_time_ms(HEARTBEAT_TIME_MS);
    next_status_update = make_timeout_time_ms(FIRMWARE_STATUS_TIME_MS);
}

/**
 * @brief Ticks all ROS related code
 */
static void tick_ros_tasks() {
    // Note that this is the *ONLY* function that any ros functions should be in, other than the executor
    // And in the executor, it should be the minimal amount of code to store the data received and get out

    // This is due to ROS timeouts/latency. In the event any reliable packet fails, it will block for the
    // whole 30 ms timeout. In order to ensure the 50ms minimum watchdog latency is met, there can only
    // be one timeout per loop. So, if any code errors for any reason, it should immediately return with an error
    // As this function should be  the only place with ROS code, it will exit before trying again, thus ensuring
    // that only 1 timeout occurs per safety tick.

    // If this is not followed, then the watchdog will reset if multiple timeouts occur within one tick

    #ifdef MICRO_ROS_TRANSPORT_CAN
    uint8_t client_id = CAN_BUS_CLIENT_ID;
    #else
    uint8_t client_id = 1;
    #endif

    if (timer_ready(&next_heartbeat, HEARTBEAT_TIME_MS, true)) {
        // RCSOFTRETVCHECK is used as important logs should occur within ros.c,
        RCSOFTRETVCHECK(ros_heartbeat_pulse(client_id));
    }

    if (timer_ready(&next_status_update, FIRMWARE_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_firmware_status(client_id));
    }

    // TODO: Put any additional ROS tasks added here
}

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

static void tick_background_tasks() {
    #if MICRO_ROS_TRANSPORT_CAN
    canbus_tick();

    if (timer_ready(&next_led_update, LED_UPTIME_INTERVAL_MS, false)) {
        led_network_online_set(canbus_check_online());
    }
    #else
    // Update the LED (so it can alternate between colors if a fault is present)
    // This is only required if CAN transport is disabled, as the led_network_online_set will update the LEDs for us
    if (timer_ready(&next_led_update, LED_UPTIME_INTERVAL_MS, false)) {
        led_update_pins();

        if (get_bootsel_button()) {
            dynamixel_set_target_position(1, 500);
        } else {
            dynamixel_set_target_position(1, 2048);
        }
    }
    #endif

    // TODO: Put any code that should periodically occur here
}

void on_dynamixel_error(uint8_t error) {
    printf("ERROR CALLBACK: %d\n", error);
    return;
}

void on_dynamixel_event(enum dynamixel_event event, dynamixel_id id) {
    switch (event) {
        case DYNAMIXEL_EVENT_PING:
            printf("Received ping from %d\n", id);
            break;
        case DYNAMIXEL_EVENT_EEPROM_READ: {
            struct dynamixel_eeprom eeprom;
            dynamixel_get_eeprom(id, &eeprom);

            printf("--- Servo %d EEPROM ---\n", id);
            printf("  Model Number: %d\n", eeprom.model_num);
            printf("  Model Info: %d\n", eeprom.model_info);
            printf("  Firmware Version: %d\n", eeprom.firmware_version);
            printf("  ID: %d\n", eeprom.id);
            printf("  Baud Rate: %d\n", eeprom.baud_rate);
            printf("  Return Delay Time: %d\n", eeprom.return_delay_time);
            printf("  Drive Mode: %d\n", eeprom.drive_mode);
            printf("  Operating Mode: %d\n", eeprom.operating_mode);
            printf("  Secondary ID: %d\n", eeprom.secondary_id);
            printf("  Protocol: %d\n", eeprom.protocol_type);
            printf("  Homing Offset: %d\n", eeprom.homing_offset);
            printf("  Temperature Limit: %d\n", eeprom.temperature_limit);
            printf("  Min Voltage Limit: %d\n", eeprom.min_voltage_limit);
            printf("  Max Voltage Limit: %d\n", eeprom.max_voltage_limit);
            printf("  PWM Limit: %d\n", eeprom.pwm_limit);
            printf("  Velocity Limit: %d\n", eeprom.velocity_limit);
            printf("  Min Position Limit: %d\n", eeprom.min_position_limit);
            printf("  Max Position Limit: %d\n", eeprom.max_position_limit);
            printf("  Startup Config: %d\n", eeprom.startup_config);
            printf("-------------------\n");

            break;
        }
        case DYNAMIXEL_EVENT_RAM_READ: {
            struct dynamixel_ram ram;
            dynamixel_get_ram(id, &ram);

            printf("--- Servo %d RAM ---\n", id);
            printf("  Torque Enable: %d\n", ram.torque_enable);
            printf("  Hardware Error Status: %d\n", ram.hardware_error_status);
            printf("  Goal Position: %d\n", ram.goal_position);
            printf("  Present Position: %d\n", ram.present_position);
            printf("  Present Velocity: %d\n", ram.present_velocity);
            printf("-------------------\n");
            break;
        }
        default:
            printf("Receieved unknown dynamixel event %d for ID %d\n", event, id);
            break;
    }
}

int main() {
    // Initialize stdio
    #ifdef MICRO_ROS_TRANSPORT_USB
    // The USB transport is special since it initializes stdio for you already
    transport_usb_serial_init_early();
    #else
    stdio_init_all();
    #endif
    LOG_INFO("%s", FULL_BUILD_TAG);

    sleep_ms(2000);
    // Perform all initializations
    // NOTE: Safety must be the first thing up after stdio, so the watchdog will be enabled
    safety_setup();
    led_init();
    ros_rmw_init_error_handling();
    // TODO: Put any additional hardware initialization code here
    printf("Stuff setup... initializing servo\n");
    sleep_ms(100);
    uint8_t servos[] = {1};
    dynamixel_init(servos, 1, on_dynamixel_error, on_dynamixel_event);
    printf("Dynamixel init complete\n");
    sleep_ms(100);
    // dynamixel_set_id(1, 3); // Change ID from 1 to 3

    dynamixel_enable_torque(1, true);

    // Initialize ROS Transports
    // TODO: If a transport won't be needed for your specific build (like it's lacking the proper port), you can remove it
    #ifdef MICRO_ROS_TRANSPORT_CAN
    uint can_id = CAN_BUS_CLIENT_ID;
    if (!transport_can_init(can_id)) {
        // No point in continuing onwards from here, if we can't initialize CAN hardware might as well panic and retry
        panic("Failed to initialize CAN bus hardware!");
    }
    #endif

    #ifdef MICRO_ROS_TRANSPORT_USB
    transport_usb_init();
    #endif


    // Enter main loop
    // This is split into two sections of timers
    // Those running with ROS, and those in the background
    // Note that both types of timers will need to conform to the minimal delay time, as there is around
    //   20ms of time worst case before the watchdog fires (as the ROS timeout is 30ms)
    // Meaning, don't block, either poll it in the background task or send it to an interrupt
    bool ros_initialized = false;
    while(true) {
        // Do background tasks
        tick_background_tasks();

        // Handle ROS state logic
        if(is_ros_connected()) {
            if(!ros_initialized) {
                LOG_INFO("ROS connected");

                if(ros_init() == RCL_RET_OK) {
                    ros_initialized = true;
                    led_ros_connected_set(true);
                    safety_init();
                    start_ros_timers();
                } else {
                    LOG_ERROR("ROS failed to initialize.");
                    ros_fini();
                }
            } else {
                ros_spin_executor();
                tick_ros_tasks();
            }
        } else if(ros_initialized){
            LOG_INFO("Lost connection to ROS")
            ros_fini();
            safety_deinit();
            led_ros_connected_set(false);
            ros_initialized = false;
        } else {
            if (time_reached(next_connect_ping)) {
                ros_ping();
                next_connect_ping = make_timeout_time_ms(UROS_CONNECT_PING_TIME_MS);
            }
        }

        // Tick safety
        safety_tick();
    }

    return 0;
}