#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "actuator_i2c/interface.h"

#include "build_version.h"

#define I2C_LOG(fmt, ...) printf(("[i2c] " fmt), ##__VA_ARGS__)

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

actuator_i2c_status_t actuator_status = {
    .firmware_status.firmware_version = 0,
    .firmware_status.fault_present = false,
    .claw_state = CLAW_STATE_UNKNOWN_POSITION,
    .torpedo1_state = TORPEDO_STATE_UNITIALIZED,
    .torpedo2_state = TORPEDO_STATE_UNITIALIZED,
    .dropper1_state = DROPPER_STATE_UNITIALIZED,
    .dropper2_state = DROPPER_STATE_UNITIALIZED,
};

#define has_irq_pending(i2c_inst, irq_name) (i2c_get_hw(i2c_inst)->raw_intr_stat & I2C_IC_RAW_INTR_STAT_##irq_name##_BITS)

static bool i2c_process_incoming_command(i2c_inst_t *i2c, actuator_i2c_cmd_t *cmd) {

}

static void i2c_respond_with_data(i2c_inst_t *i2c, uint8_t *data, size_t len) {
    // This function should only be called when there is an active read request
    assert(has_irq_pending(i2c, RD_REQ));
    i2c_get_hw(i2c)->clr_rd_req;

    if (has_irq_pending(i2c, TX_ABRT) && i2c_get_hw(i2c)->tx_abrt_source & I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_BITS) {
        i2c_get_hw(i2c)->clr_tx_abrt;
        uint32_t flush_count = (i2c_get_hw(i2c)->tx_abrt_source >> I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_LSB);
        I2C_LOG("Unexpected %d bytes in tx fifo at start of transmission\n", flush_count);
    }

    // Send data until the bus is released
    size_t bytes_sent = 0;
    while (!has_irq_pending(i2c, RX_DONE)) {
        if (i2c_get_write_available(i2c) && bytes_sent < len) {
            i2c_get_hw(i2c)->data_cmd = (uint32_t)data[bytes_sent++];
            i2c_get_hw(i2c)->clr_rd_req;
        }

        // Exit early if the bus is 
        if (bytes_sent == len && has_irq_pending(i2c, RD_REQ)) {
            I2C_LOG("More bytes requested to be read than expected. Exiting early\n");
            break;
        }

        if (has_irq_pending(i2c, TX_ABRT)) {
            i2c_get_hw(i2c)->clr_tx_abrt;
            I2C_LOG("Abort occured during transmission! Abort Reg: 0x%08x - Exiting early\n", i2c_get_hw(i2c)->tx_abrt_source);
            break;
        }

        tight_loop_contents();
    }
    i2c_get_hw(i2c)->clr_rx_done;

    if (bytes_sent != len) {
        I2C_LOG("Incomplete Transmission: %d bytes sent out of %d requested\n", bytes_sent, len);
    }
}

/**
 * @brief Attempts to retreive the next command from the i2c bus.
 * On success the command will be written to cmd. Note that this may be written to on failure, but the value is meaningless.
 * This function may return in failure for a variety of reasons, and does not explicitly mean a timeout.
 * A false return value could include a malformed message being received, a status message being succesfully processed, or a checksum failure.
 * 
 * @param i2c The i2c hardware to retrieve the next command on
 * @param cmd The output to write the command to
 */
void i2c_get_next_command(i2c_inst_t *i2c, actuator_i2c_cmd_t *cmd) {
    uint8_t *raw_cmd = (uint8_t*)cmd;
    const size_t max_cmd_size = sizeof(*cmd);

    i2c_get_hw(i2c)->enable = 1;
    while (!(i2c_get_hw(i2c)->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS)) {tight_loop_contents();}

    while (true) {
        if (has_irq_pending(i2c, RD_REQ)) {
            actuator_status.crc8 = actuator_i2c_crc8_calc_status(&actuator_status);
            uint8_t *raw_status_msg = (uint8_t*)(&actuator_status);
            size_t status_msg_size = sizeof(actuator_status);

            i2c_respond_with_data(i2c, raw_status_msg, status_msg_size);
        }

        if (i2c_get_hw(i2c)->rxflr) {
            if (i2c_process_incoming_command(i2c, cmd)) {
                break;
            }
        }

        tight_loop_contents();
    }

    i2c_get_hw(i2c)->enable = 0;
    while (i2c_get_hw(i2c)->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) {tight_loop_contents();}
}

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    printf("%s\n", FULL_BUILD_TAG);

    bool value = true;

    i2c_init(BUILTIN_I2C_HW, 200000);
    i2c_set_slave_mode(BUILTIN_I2C_HW, true, ACTUATOR_I2C_ADDR);
    i2c_get_hw(BUILTIN_I2C_HW)->con |= I2C_IC_CON_STOP_DET_IFADDRESSED_BITS;
    gpio_set_function(BUILTIN_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BUILTIN_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BUILTIN_SDA_PIN);
    gpio_pull_up(BUILTIN_SCL_PIN);

    actuator_i2c_cmd_t my_cmd;

    printf("Waiting for data...\n");
    while (true) {
        if (i2c_get_next_command_until(BUILTIN_I2C_HW, &my_cmd, make_timeout_time_ms(1000))) {
            printf("New Command: %d\n", my_cmd.cmd_id);

            size_t command_size = 2;
            switch (my_cmd.cmd_id) {
                case ACTUATOR_CMD_CLAW_TIMING:
                    printf("Claw Timings Command - Open Time: %d ms - Close Time: %d ms", my_cmd.data.claw_timing.open_time_ms, my_cmd.data.claw_timing.close_time_ms);
                    command_size += ACTUATOR_CMD_CLAW_TIMING_LENGTH;
                    break;
                case ACTUATOR_CMD_TEST:
                    printf("Test Command - Data 1: 0x%08x - Data 2: 0x%08x", my_cmd.data.test.data1, my_cmd.data.test.data2);
                    command_size += ACTUATOR_CMD_TEST_LENGTH;
                    break;
                default:
                    printf("Unknown command!\n");
            }

            printf("Raw Command Data: ");
            uint8_t *raw_data = (uint8_t*)(&my_cmd);
            for (int i = 0; i < command_size; i++) {
                printf("%s0x%02x", (i ==0 ? "" : ", "), raw_data[i]);
            }
            printf("\n\nWaiting for data...\n");
        }
    }
    return 0;
}