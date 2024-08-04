#include "ledc.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "titan/debug.h"

#include <stdlib.h>
#include <string.h>

#define LEDC_SPI_INST __CONCAT(spi, LEDC_SPI)

#define LEDC1 0
#define LEDC2 1
#define OPCODE_WRITE 0b00
#define OPCODE_READ 0b01
#define OPCODE_READ_CLR 0b10
#define OPCODE_READ_ROM 0b11

static uint32_t spi_xfer(uint target, uint32_t data, uint8_t *gs_out) {
    uint8_t tx_packet[] = { (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF };
    uint8_t rx_packet[sizeof(tx_packet)];
    uint cs_pin = target == LEDC1 ? LEDC_NCS1_PIN : LEDC_NCS2_PIN;

    gpio_put(cs_pin, 0);
    busy_wait_us(1);
    if (spi_write_read_blocking(LEDC_SPI_INST, tx_packet, rx_packet, sizeof(tx_packet)) != 4) {
        // TODO: Something a little less agressive than this
        panic("LEDC SPI Xfer Failure!");
    }
    busy_wait_us(1);
    gpio_put(cs_pin, 1);

    if (gs_out) {
        *gs_out = rx_packet[0];
    }

    return ((uint32_t) (rx_packet[1]) << 16) | ((uint32_t) (rx_packet[2]) << 8) | rx_packet[3];
}

// Wrappers around SPI logic
#define spi_xfer_cmd(target, opcode, addr, data, gs_out)                                                               \
    spi_xfer((target), ((opcode) << 30) | (((addr) & 0x3F) << 24) | ((data) & 0xFFFFFF), (gs_out))
#define spi_read(target, addr, gs_out) spi_xfer_cmd(target, OPCODE_READ, addr, 0, gs_out)
#define spi_read_clr(target, addr, gs_out) spi_xfer_cmd(target, OPCODE_READ_CLR, addr, 0, gs_out)
#define spi_read_rom(target, addr, gs_out) spi_xfer_cmd(target, OPCODE_READ_ROM, addr, 0, gs_out)
#define spi_write(target, addr, data, gs_out) spi_xfer_cmd(target, OPCODE_WRITE, addr, data, gs_out)

static void print_global_status(FILE *fout, uint8_t gs) {
    fprintf(fout, "==Global Status Set Bits==\n");
    if ((gs >> 7) & 1) {
        fprintf(fout, " - Global Status Bit Not\n");
    }
    if ((gs >> 6) & 1) {
        fprintf(fout, " - Reset Bit\n");
    }
    if ((gs >> 5) & 1) {
        fprintf(fout, " - SPI Err\n");
    }
    if ((gs >> 4) & 1) {
        fprintf(fout, " - Functional Err 2\n");
    }
    if ((gs >> 3) & 1) {
        fprintf(fout, " - Functional Err 1\n");
    }
    if ((gs >> 2) & 1) {
        fprintf(fout, " - Device Error\n");
    }
    if ((gs >> 1) & 1) {
        fprintf(fout, " - Global Warning\n");
    }
    if ((gs >> 0) & 1) {
        fprintf(fout, " - Fail Safe\n");
    }
    fprintf(fout, "\n");
}

static int parse_int(const char *str, long long *val_out) {
    char *end;
    long long val;
    if (str[0] == '0' && str[1] == 'x') {
        val = strtoll(&str[2], &end, 16);
    }
    else {
        val = strtoll(str, &end, 10);
    }
    if (*end != 0 || end == str) {
        return 1;
    }
    *val_out = val;
    return 0;
}

#define parse_int_with_bounds(str, var_out, min_bounds, max_bounds)                                                    \
    do {                                                                                                               \
        long long val_tmp;                                                                                             \
        if (parse_int(str, &val_tmp)) {                                                                                \
            fprintf(fout, "Invalid Decimal Value Specified: '%s'\n", str);                                             \
            return 1;                                                                                                  \
        }                                                                                                              \
        if (val_tmp > (max_bounds) || val_tmp < (min_bounds)) {                                                        \
            fprintf(fout, "Provided number '%s' out of bounds! Must be between %lld - %lld\n", str,                    \
                    (long long) (min_bounds), (long long) (max_bounds));                                               \
            return 1;                                                                                                  \
        }                                                                                                              \
        (var_out) = val_tmp;                                                                                           \
    } while (0)

static int ledc_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc < 4) {
        fprintf(fout, "Usage: @ledc [cmd] [target] [addr] [data (if write)]\n");
        return 1;
    }

    const char *cmd = argv[1];

    const char *target_str = argv[2];
    uint target;
    if (target_str[0] == '1' && target_str[1] == '\0') {
        target = 1;
    }
    else if (target_str[0] == '2' && target_str[1] == '\0') {
        target = 2;
    }
    else {
        fprintf(fout, "Invalid Target: Must be either '1' or '2', not '%s'\n", target_str);
        return 1;
    }

    uint addr;
    parse_int_with_bounds(argv[3], addr, 0, 0x3F);

    if (!strcmp(cmd, "read")) {
        uint8_t gs;
        uint val = spi_read(target, addr, &gs);
        print_global_status(fout, gs);
        fprintf(fout, "Register %d: 0x%06X\n", addr, val);
    }
    else if (!strcmp(cmd, "rdclr")) {
        uint8_t gs;
        uint val = spi_read_clr(target, addr, &gs);
        print_global_status(fout, gs);
        fprintf(fout, "Register %d: 0x%06X\n", addr, val);
        fprintf(fout, "Register Cleared\n");
    }
    else if (!strcmp(cmd, "rom")) {
        uint8_t gs;
        uint val = spi_read_rom(target, addr, &gs);
        print_global_status(fout, gs);
        fprintf(fout, "ROM @0x%02X: 0x%02X\n", addr, val);
    }
    else if (!strcmp(cmd, "write")) {
        if (argc < 5) {
            fprintf(fout, "Missing Data Parameter\n");
            return 1;
        }

        uint data;
        parse_int_with_bounds(argv[4], data, 0, 0xFFFFFF);

        uint8_t gs;
        spi_write(target, addr, data, &gs);
        print_global_status(fout, gs);
        fprintf(fout, "Register %d Written\n", addr);
    }
    else {
        fprintf(fout, "Invalid Command: '%s'\n", cmd);
        return 1;
    }
    return 0;
}

void ledc_init(void) {
    // SPI Init
    bi_decl_if_func_used(bi_3pins_with_func(LEDC_MISO_PIN, LEDC_MOSI_PIN, LEDC_SCK_PIN, LEDC_SPI));
    spi_init(LEDC_SPI_INST, 4000000);  // Run SPI at 4 MHz
    spi_set_format(LEDC_SPI_INST,      // SPI instance
                   8,                  // Number of bits per transfer
                   0,                  // Polarity (CPOL)
                   0,                  // Phase (CPHA)
                   SPI_MSB_FIRST);     // MSB First
    gpio_set_function(LEDC_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LEDC_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LEDC_MOSI_PIN, GPIO_FUNC_SPI);

    // Configure the CS pins
    bi_decl_if_func_used(bi_1pin_with_name(LEDC_NCS1_PIN, "LEDC 1 nCS Pin"));
    bi_decl_if_func_used(bi_1pin_with_name(LEDC_NCS2_PIN, "LEDC 2 nCS Pin"));
    gpio_init(LEDC_NCS1_PIN);
    gpio_put(LEDC_NCS1_PIN, 1);
    gpio_set_dir(LEDC_NCS1_PIN, GPIO_OUT);
    gpio_init(LEDC_NCS2_PIN);
    gpio_put(LEDC_NCS2_PIN, 1);
    gpio_set_dir(LEDC_NCS2_PIN, GPIO_OUT);

    // TODO: Sam what frequency do you want for PWM_CLK?

    debug_remote_cmd_register("ledc", "[cmd] [target] [addr] [data (if write)]",
                              "Issues the command to the target LED controller\n"
                              "Target must be either '1' or '2'\n"
                              "===Commands===\n"
                              "  read\tReads the specific register\n"
                              "  write\tWrites the specific register\n"
                              "  rdclr\tReads/clears the specified register\n"
                              "  rom\tReads the specified ROM address",
                              ledc_cmd_cb);
}
