// NOTE THIS FILE IS INCLUDED IN ledc_commands.c
// THIS IS SEPARATED INTO A DIFFERENT FILE TO KEEP CLUTTER OUT

static int get_controller_status(__unused size_t argc, __unused const char *const *argv, FILE *fout) {
    uint8_t gs;

    uint val1_1 = spi_read(LEDC1, 0x01, &gs);
    uint val2_1 = spi_read(LEDC1, 0x02, &gs);
    uint val6_1 = spi_read(LEDC1, 0x06, &gs);
    for (int count = 0; count < 100000; count++) {
    }
    uint val1_2 = spi_read(LEDC2, 0x01, &gs);
    uint val2_2 = spi_read(LEDC2, 0x02, &gs);
    uint val6_2 = spi_read(LEDC2, 0x06, &gs);

    fprintf(fout, "\nController 1 Status \n");

    if (val1_1 & (1 << 1)) {
        fprintf(fout, "Unlocked - UNLOCK: 1 \n");
    }
    else {
        fprintf(fout, "Locked - UNLOCK: 0 \n");
    }

    if (val2_1 & (1 << 3)) {
        fprintf(fout, "In Standby - GOSTBY: 1 \n");
    }
    else {
        fprintf(fout, "Woke Up, Not in Standby  -  GOSTBY: 0 \n");
    }

    if (val2_1 & (1 << 2)) {
        fprintf(fout, "Enabled - EN: 1 \n");
    }
    else {
        fprintf(fout, "Not Enabled / Limp Home -  EN: 0 \n");
    }

    if ((val6_1 & (1 << 16)) && (val6_1 & (1 << 15))) {
        fprintf(fout, "Watchdog Timer .75+ \n");
    }
    else if ((val6_1 & (1 << 16)) && !(val6_1 & (1 << 15))) {
        fprintf(fout, "Watchdog Timer .50+ \n");
    }
    else if (!(val6_1 & (1 << 16)) && (val6_1 & (1 << 15))) {
        fprintf(fout, "Watchdog Timer .25+ \n");
    }
    else {
        fprintf(fout, "Watchdog Timer .0+ \n");
    }

    fprintf(fout, "\nController 2 Status \n");

    if (val1_2 & (1 << 1)) {
        fprintf(fout, "Unlocked - UNLOCK: 1 \n");
    }
    else {
        fprintf(fout, "Locked - UNLOCK: 0 \n");
    }

    if (val2_2 & (1 << 3)) {
        fprintf(fout, "In Standby - GOSTBY: 1 \n");
    }
    else {
        fprintf(fout, "Woke Up, Not in Standby  -  GOSTBY: 0 \n");
    }

    if (val2_2 & (1 << 2)) {
        fprintf(fout, "Enabled - EN: 1 \n");
    }
    else {
        fprintf(fout, "Not Enabled / Limp Home -  EN: 0 \n");
    }

    if ((val6_2 & (1 << 16)) && (val6_2 & (1 << 15))) {
        fprintf(fout, "Watchdog Timer .75+ \n");
    }
    else if ((val6_2 & (1 << 16)) && !(val6_2 & (1 << 15))) {
        fprintf(fout, "Watchdog Timer .50+ \n");
    }
    else if (!(val6_2 & (1 << 16)) && (val6_2 & (1 << 15))) {
        fprintf(fout, "Watchdog Timer .25+ \n");
    }
    else {
        fprintf(fout, "Watchdog Timer .0+ \n");
    }

    fprintf(fout, "Sysclk Freq: %ld Hz\n", clock_get_hz(clk_sys));

    return 0;
}

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
        target = LEDC1;
    }
    else if (target_str[0] == '2' && target_str[1] == '\0') {
        target = LEDC2;
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

static int al_read_temp_cb(__unused size_t argc, __unused const char *const *argv, FILE *fout) {
    float temperature = al_read_temp();

    fprintf(fout, "Temperature: %f \n", temperature);

    return 0;
}

void register_canmore_commands() {
    debug_remote_cmd_register("leds", "", "Read and decode status of led controllers.\n", get_controller_status);
    debug_remote_cmd_register("ledc", "[cmd] [target] [addr] [data (if write)]",
                              "Issues the command to the target LED controller\n"
                              "Target must be either '1' or '2'\n"
                              "===Commands===\n"
                              "  read\tReads the specific register\n"
                              "  write\tWrites the specific register\n"
                              "  rdclr\tReads/clears the specified register\n"
                              "  rom\tReads the specified ROM address",
                              ledc_cmd_cb);
    debug_remote_cmd_register("temp", "", "Read Temperature on LEDS.\n", al_read_temp_cb);
}
