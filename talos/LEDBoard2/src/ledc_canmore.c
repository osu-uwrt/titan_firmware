#include "ledc_canmore.h"

#include "ledc_commands.h"
#include "ledc_io.h"
#include "ledc_registers.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

int32_t read_temp_cb(size_t argc, const char *const *argv, FILE *fout) {
    float temp = read_temp();
    fprintf(fout, "Aluminum Board Temperature: [%.4f]\n", temp);
}

int32_t ledc_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {}

int32_t get_ledc_status_cb(size_t argc, const char *const *argv, FILE *fout) {
    cr1_fields_t cr1_ledc1 = decode_cr1(decode_register(spi_read(LEDC1, CR1_ADDRESS)));
    cr1_fields_t cr1_ledc2 = decode_cr1(decode_register(spi_read(LEDC2, CR1_ADDRESS)));

    cr2_fields_t cr2_ledc1 = decode_cr2(decode_register(spi_read(LEDC1, CR2_ADDRESS)));
    cr2_fields_t cr2_ledc2 = decode_cr2(decode_register(spi_read(LEDC2, CR2_ADDRESS)));

    cr1_t cr1_ledc1_raw = decode_register(spi_read(LEDC1, CR1_ADDRESS));
    cr1_t cr1_ledc2_raw = decode_register(spi_read(LEDC2, CR1_ADDRESS));

    cr2_t cr2_ledc1_raw = decode_register(spi_read(LEDC1, CR2_ADDRESS));
    cr2_t cr2_ledc2_raw = decode_register(spi_read(LEDC2, CR2_ADDRESS));

    spi_frame_t cr1_frame_ledc1 = spi_read(LEDC1, CR1_ADDRESS);
    spi_frame_t cr1_frame_ledc2 = spi_read(LEDC2, CR1_ADDRESS);

    fprintf(fout, "controller 1 cr1: [%X]\n", cr1_ledc1_raw);
    fprintf(fout, "controller 2 cr1: [%X]\n", cr1_ledc2_raw);

    fprintf(fout, "controller 1 cr2: [%X]\n", cr2_ledc1_raw);
    fprintf(fout, "controller 2 cr2: [%X]\n", cr2_ledc2_raw);

    fprintf(fout, "controller 1 cr1 frame: [%X]\n", cr1_frame_ledc1);
    fprintf(fout, "controller 2 cr1 frame: [%X]\n", cr1_frame_ledc2);
    fprintf(fout, "CONTROLLER 1 STATUS\n");
    if (cr1_ledc1.unlock) {
        fprintf(fout, "Unlocked - UNLOCK: 1 \n");
    }
    else {
        fprintf(fout, "Locked - UNLOCK: 0 \n");
    }

    if (cr2_ledc1.gostby) {
        fprintf(fout, "In Standby - GOSTBY: 1 \n");
    }
    else {
        fprintf(fout, "Woke Up, Not in Standby  -  GOSTBY: 0 \n");
    }

    if (cr2_ledc1.enable) {
        fprintf(fout, "Enabled - EN: 1 \n");
    }
    else {
        fprintf(fout, "Not Enabled / Limp Home -  EN: 0 \n");
    }

    fprintf(fout, "CONTROLLER 2 STATUS\n");
    if (cr1_ledc2.unlock) {
        fprintf(fout, "Unlocked - UNLOCK: 1 \n");
    }
    else {
        fprintf(fout, "Locked - UNLOCK: 0 \n");
    }

    if (cr2_ledc2.gostby) {
        fprintf(fout, "In Standby - GOSTBY: 1 \n");
    }
    else {
        fprintf(fout, "Woke Up, Not in Standby  -  GOSTBY: 0 \n");
    }

    if (cr2_ledc2.enable) {
        fprintf(fout, "Enabled - EN: 1 \n");
    }
    else {
        fprintf(fout, "Not Enabled / Limp Home -  EN: 0 \n");
    }
}

void canmore_serve(uint32_t argc, char **argv, FILE *out) {
    if (strcmp(argv[0], "ledc") == 0) {
    }
    else if (strcmp(argv[0], "temp") == 0) {
    }
    else if (strcmp(argv[0], "leds") == 0) {
    }
}

void set_active_cb(size_t argc, const char *const *argv, FILE *fout) {
    set_controllers_active_mode();
}

void set_peak_current_cb(size_t argc, const char *const *argv, FILE *fout) {
    set_peak_current();
}

void print_gs(FILE *fout, uint8_t gs) {
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

void register_canmore_commands() {
    debug_remote_cmd_register("leds", "", "Read and decode status of led controllers.\n", get_ledc_status_cb);
    // debug_remote_cmd_register("ledc", "[cmd] [target] [addr] [data (if write)]",
    //                           "Issues the command to the target LED controller\n"
    //                           "Target must be either '1' or '2'\n"
    //                           "===Commands===\n"
    //                           "  read\tReads the specific register\n"
    //                           "  write\tWrites the specific register\n"
    //                           "  rdclr\tReads/clears the specified register\n"
    //                           "  rom\tReads the specified ROM address",
    //                           ledc_cmd_cb);
    debug_remote_cmd_register("set_active", "", "Set both controllers into active mode.\n", set_active_cb);
    debug_remote_cmd_register("set_peak_current", "", "Set both controllers peak current.\n", set_peak_current_cb);
    debug_remote_cmd_register("temp", "", "Read Temperature on LEDS.\n", read_temp_cb);
}
