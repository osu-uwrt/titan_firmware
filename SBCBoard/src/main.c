#include "boards/talos/sbc_board.h"
#include "bq25730.h"
#include "safety_interface.h"

#include "hardware/watchdog.h"

#include "driver/async_i2c.h"
#include "driver/canbus.h"
#include "driver/led.h"
#include "micro_ros_pico/transport_can.h"
#include "pico/stdlib.h"
#include "titan/binary_info.h"
#include "titan/logger.h"
#include "titan/version.h"

static void tick_background_tasks() {}

int main() {
    // Initialize stdio
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    // Perform all initializations
    // NOTE: Safety must be the first thing up after stdio, so the watchdog will be enabled
    safety_setup();

    // FIXME: canbus_utility_fram_register_cb() ?
    async_i2c_init(BOARD_SDA_PIN, BOARD_SCL_PIN, -1, -1, 200000, 10);

    // Initialize bq25730
    bq25730_init(BOARD_I2C);

    // Configure BQ25730
    bq25730_wait_until_configured();
    gpio_init(25);
    gpio_set_dir(25, GPIO_IN);
    // Main Loop
    while (true) {
        safety_tick();
        // printf("\nSystem Voltage: \n");
        // bq25730_start_read_system_voltage();
        sleep_ms(500);
        printf("BQ25730 State: %s\n", bq25730_state_to_string(bq25730_get_state()));
        printf("Charge OK: %d\n", gpio_get(25));
    }

    // Initialize USB-PD chip
    // usbpd_init();

    // Initialize screen
    // screen_init();

    uint can_id = CAN_BUS_CLIENT_ID;
    bi_decl_if_func_used(bi_client_id(CAN_BUS_CLIENT_ID));
    if (!canbus_init(can_id)) {  // FIXME: should it be transport_can_init()?
        // No point in continuing onwards from here, if we can't initialize CAN hardware might as well panic and retry
        panic("Failed to initialize CAN bus hardware!");
    }
    while (true) {
        tick_background_tasks();


        safety_tick();
    }

    return 0;
}
