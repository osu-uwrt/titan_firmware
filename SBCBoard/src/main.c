#include "boards/talos/sbc_board.h"
#include "bq25730.h"
#include "tps25750.h"
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

    // Initialize tps25750
    tps25750_init(BOARD_I2C);

    // Initialize screen
    // screen_init();

    bq25730_start_write_enable_low_power_mode();
    sleep_ms(500);

    bq25730_start_write_ADC_option();
    sleep_ms(500);

    while (true) {
        printf("\nBQ Manufacturer ID: \n");
        bq25730_start_read_manufacturer_id();
        sleep_ms(500);
        printf("\nBQ System Voltage: \n");
        bq25730_start_read_system_voltage();
        sleep_ms(500);
        printf("\nTPS MODE: \n");
        tps25750_start_read_mode();
        sleep_ms(500);
        safety_tick();
        sleep_ms(500);
    }

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
