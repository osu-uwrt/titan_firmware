#include "ledc.h"
#include <math.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "titan/debug.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define LEDC_SPI_INST __CONCAT(spi, LEDC_SPI)

#define LEDC1 0
#define LEDC2 1
#define OPCODE_WRITE 0b00
#define OPCODE_READ 0b01
#define OPCODE_READ_CLR 0b10
#define OPCODE_READ_ROM 0b11

#define THERMISTOR_SERIES_RESISTANCE 10000.0
#define THERMISTOR_V_REF 3.3
#define THERMISTOR_R_25 22000.0
#define THERMISTOR_B_25_85 3730.0
#define THERMISTOR_NOMINAL_TEMP 298.15
#define OPERATING_TEMPERATURE_MAX 50 //celcius

#define PULSE_PERIOD 50

repeating_timer_t watchdog_timer;
uint8_t watchdog_state = false;
uint32_t called_count = 0;
uint32_t write_fail = 0;
uint32_t previous_val = 0;

repeating_timer_t pulse_end_timer;
uint8_t pulsing = false;


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

uint32_t correct_parity_bit(uint32_t val, bool odd_parity){
    //ensure the parity bit at the end of the 24 bit spi is correctly computed
    //ODD parity bit check

    uint32_t val_count = val;

    //count the number of ones in the 24 bit int
    int ones_count = 0;
    for(int n = 23; n>=1; n = n - 1)
    {
        if(val_count - pow(2,n) >= 0){
            ones_count++;
            val_count = val_count - pow(2,n);
        }
    }

    if(odd_parity){
        if(!(ones_count % 2)){
            return (val | (1<<0));
        }

        return (val & 4294967294);
    }

    if((ones_count % 2)){
        return (val | (1<<0));
    }
    return (val & 4294967294);

}

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

float read_temp(){
    //read led al board thermistor

    adc_select_input(0);
    const float convert_to_voltage_factor = 3.3f / (1 << 12);
    float voltage = adc_read()  * convert_to_voltage_factor;

    //calculate thermistor resistance
    float resistance = THERMISTOR_SERIES_RESISTANCE * (THERMISTOR_V_REF * voltage) / (THERMISTOR_V_REF + voltage);

    float temperature = 1 / (log(resistance / THERMISTOR_R_25) / THERMISTOR_B_25_85 + 1 / THERMISTOR_NOMINAL_TEMP) - 273.0;

    return temperature;
}

static int read_temp_cb(size_t argc, const char *const *argv, FILE *fout){
    float temperature = read_temp();

    fprintf(fout, "Temperature: %f \n", temperature);

    return 0;
}

static int get_controller_status(size_t argc, const char *const *argv, FILE *fout){

    uint8_t gs;

    uint val1_1 = spi_read(1, 0x01, &gs);
    uint val2_1 = spi_read(1, 0x02, &gs);
    uint val6_1 = spi_read(1, 0x06, &gs);
    for(int count = 0; count < 100000; count++){}
    uint val1_2 = spi_read(2, 0x01, &gs);
    uint val2_2 = spi_read(2, 0x02, &gs);
    uint val6_2 = spi_read(2, 0x06, &gs);

    fprintf(fout, "\nController 1 Status \n");

    if(val1_1 & (1 << 1)){
        fprintf(fout, "Unlocked - UNLOCK: 1 \n");
    } else {
        fprintf(fout, "Locked - UNLOCK: 0 \n");
    }

    if(val2_1 & (1 << 3)){
        fprintf(fout, "In Standby - GOSTBY: 1 \n");
    } else {
        fprintf(fout, "Woke Up, Not in Standby  -  GOSTBY: 0 \n");
    }

    if(val2_1 & (1 << 2)){
        fprintf(fout, "Enabled - EN: 1 \n");
    } else {
        fprintf(fout, "Not Enabled / Limp Home -  EN: 0 \n");
    }

    if((val6_1 & (1 << 16)) && (val6_1 & (1 << 15))){
        fprintf(fout, "Watchdog Timer .75+ \n");
    } else if ((val6_1 & (1 << 16)) && !(val6_1 & (1 << 15))){
        fprintf(fout, "Watchdog Timer .50+ \n");
    } else if(!(val6_1 & (1 << 16)) && (val6_1 & (1 << 15))){
        fprintf(fout, "Watchdog Timer .25+ \n");
    } else{
        fprintf(fout, "Watchdog Timer .0+ \n");
    }

    fprintf(fout, "\nController 2 Status \n");

    if(val1_2 & (1 << 1)){
        fprintf(fout, "Unlocked - UNLOCK: 1 \n");
    } else {
        fprintf(fout, "Locked - UNLOCK: 0 \n");
    }

    if(val2_2 & (1 << 3)){
        fprintf(fout, "In Standby - GOSTBY: 1 \n");
    } else {
        fprintf(fout, "Woke Up, Not in Standby  -  GOSTBY: 0 \n");
    }

    if(val2_2 & (1 << 2)){
        fprintf(fout, "Enabled - EN: 1 \n");
    } else {
        fprintf(fout, "Not Enabled / Limp Home -  EN: 0 \n");
    }

    if((val6_2 & (1 << 16)) && (val6_2 & (1 << 15))){
        fprintf(fout, "Watchdog Timer .75+ \n");
    } else if ((val6_2 & (1 << 16)) && !(val6_2 & (1 << 15))){
        fprintf(fout, "Watchdog Timer .50+ \n");
    } else if(!(val6_2 & (1 << 16)) && (val6_2 & (1 << 15))){
        fprintf(fout, "Watchdog Timer .25+ \n");
    } else{
        fprintf(fout, "Watchdog Timer .0+ \n");
    }

    fprintf(fout, "Sysclk Freq: %ld Hz\n", clock_get_hz(clk_sys));

    return 0;

}

static bool satisfy_watchdog(struct repeating_timer *t){
    uint8_t gs;
    
    uint32_t val = 45;

    if(watchdog_state){
        spi_write(1,0x04,0,&gs);
        spi_write(2,0x04,0,&gs);

        val = spi_read(1, 0x04, &gs);

    } else {
        spi_write(1,0x04,8388609, &gs);
        spi_write(2,0x04,8388609, &gs);

        val = spi_read(1, 0x04, &gs);

    }

    if(val == previous_val){
        write_fail++;
    }

    previous_val = val;

    watchdog_state = !watchdog_state;

    called_count++;

    return true;
}

static int enable_controllers(size_t argc, const char *const *argv, FILE *fout){
    uint8_t gs;

    //set control regisiter 1

    //unlocked
    sleep_ms(1);
    uint32_t val = spi_read(1,0x01, &gs);
    val = val | (1 << 3);
    val = val | (1 << 2);
    val = correct_parity_bit(val | (1 << 1), false);
    sleep_ms(1);
    spi_write(1, 0x01, val, &gs);
    sleep_ms(1);
    uint32_t val_read = spi_read(1,0x01, &gs);
    fprintf(fout, "Controller 1 Register 1: Setting: 0x%06X   Reading: 0x%06X \n", val, val_read);

    sleep_ms(1);
    val = spi_read(2,0x01, &gs);
    val = val | (1 << 3);
    val = val | (1 << 2);
    val = correct_parity_bit(val | (1 << 1), true);
    sleep_ms(1);
    spi_write(2, 0x01, val, &gs);

    //set control regisiter 2

    //enabled and gostby
    sleep_ms(1);
    val = spi_read(1,0x02, &gs);
    val = val | (1 << 3);
    val = correct_parity_bit(val | (1 << 2), false);
    sleep_ms(1);
    spi_write(1, 0x02, val, &gs);
    sleep_ms(1);
    val_read = spi_read(1,0x02, &gs);
    fprintf(fout, "Controller 1 Register 2: Setting: 0x%06X   Reading: 0x%06X \n", val, val_read);

    sleep_ms(1);
    val = spi_read(2,0x02, &gs);
    val = val | (1 << 3);
    val = correct_parity_bit(val | (1 << 2), true);
    sleep_ms(1);
    spi_write(2, 0x02, val, &gs);
    sleep_ms(1);

    //set control regisiter 3

    //enabled and gostby
    sleep_ms(1);
    val = spi_read(1,0x03, &gs);
    val = val | (1 << 15);
    val = val & 4294950911;
    val = val | (1 << 13);
    val = val & 4294963199;
    val = correct_parity_bit(val, true);
    sleep_ms(1);
    spi_write(1, 0x03, val, &gs);
    sleep_ms(1);
    val_read = spi_read(1,0x03, &gs);
    fprintf(fout, "Controller 1 Register 3: Setting: 0x%06X   Reading: 0x%06X \n", val, val_read);


    sleep_ms(1);
    val = spi_read(2,0x03, &gs);
    val = val | (1 << 3);
    val = correct_parity_bit(val | (1 << 2), true);
    sleep_ms(1);
    spi_write(2, 0x03, val, &gs);
    sleep_ms(1);

    watchdog_state = !watchdog_state;

    // val = spi_read(1,0x04,&gs);
    // uint8_t flipped = true;

    // while(flipped){
    //     sleep_ms(1);
    //     if(val != spi_read(1,0x04,&gs)){
    //         flipped = false;

    //     }

    // }
    
    // uint8_t count = 0;
    // flipped = true;
    
    // for(int x = 0; x<20; x++){
    //     count++;
    //     sleep_ms(1);

    //     uint32_t temp = spi_read(1,0x04,&gs);

    //     fprintf(fout, "Period: %d Six: 0x%06X \n", count, temp);

    //     if(val == temp){
    //         flipped = false;
    //     }
    // }

    fprintf(fout, "Calls: %d \n", called_count);
    fprintf(fout, "Write Fails: %d \n", write_fail);

    return 0;
}

bool set_din(int target, int value){
    //set din with temperature protection
    if(read_temp() < OPERATING_TEMPERATURE_MAX){
        
        if(target == 1){
            gpio_put(LEDC_DIN1_PIN, value);
        }else{
            gpio_put(LEDC_DIN2_PIN, value);
        }

        return 1;
    }

    gpio_put(LEDC_DIN1_PIN, 0);
    gpio_put(LEDC_DIN2_PIN, 0);


    //return false if din was set low due to temperature
    return 0;
}

static int setdin_cb(size_t argc, const char *const *argv, FILE *fout) {
    if(argc < 2){
        fprintf(fout, "Not Enought Arguments - Please enter target and value \n");
    }

    const char *target_str = argv[1];
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

    const char *value_str = argv[2];
    uint value;
    if (value_str[0] == '0' && value_str[1] == '\0') {
        value = 0;
    }
    else if (value_str[0] == '1' && value_str[1] == '\0') {
        value = 1;
    }
    else {
        fprintf(fout, "Invalid Target: Must be either '1' or '2', not '%s'\n", target_str);
        return 1;
    }

    fprintf(fout, "Setting DIN %d to: %d \n", target, value);


    if(!set_din(target, value)){
        fprintf(fout, "Setting DIN low, overheating!\n");
    }

    return 0;
}

static int start_pulse_cb(size_t argc, const char *const *argv, FILE *fout){
    led_pulse_start();

    return 0;
}

void led_pulse_end(){
    if(pulsing == 2){
        //turn off leds and increment state
        gpio_put(LEDC_DIN1_PIN, 0);


        pulsing = 1;
        return;
    } else{
        //end cycle 

        pulsing = 0;   
        cancel_repeating_timer(&pulse_end_timer);

        return;
    }


    return;
}

void led_pulse_start(void){
    add_repeating_timer_ms(PULSE_PERIOD, led_pulse_end, NULL, &pulse_end_timer);
    
    if(!(pulsing)){
        //turn on leds and start pulsing sequence
        gpio_put(LEDC_DIN1_PIN, 1);
        pulsing = 2;
    }

    return;
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

    //provide pwm clock
    gpio_set_function(LEDC_PWM_CLK, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(LEDC_PWM_CLK);
    uint chan = pwm_gpio_to_channel(LEDC_PWM_CLK);

    // Want PWMCLK to be a square wave at 204,800 Hz
    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, 3);
    // Set channel A output high for two cycles before dropping (50% duty cycle)
    pwm_set_chan_level(slice_num, chan, 2);
    // This means that we need the slice to be clocked at 819,200 Hz
    // Compute fractioanl divider to get our target frequency
    pwm_set_clkdiv(slice_num, clock_get_hz(clk_sys) / 819200.0f);
    // Set the PWM running
    pwm_set_enabled(slice_num, true);


    gpio_init(LEDC_DIN1_PIN);
    gpio_set_dir(LEDC_DIN1_PIN, GPIO_OUT);

    gpio_init(LEDC_DIN2_PIN);
    gpio_set_dir(LEDC_DIN2_PIN, GPIO_OUT);
    gpio_put(LEDC_DIN2_PIN, 1);

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

    //init adc - for read thermistor
    adc_init();
    adc_gpio_init(26);

    //repeating timer to make watchdog happy
    add_repeating_timer_ms(3, satisfy_watchdog, NULL, &watchdog_timer);


    debug_remote_cmd_register("temp", "",
                              "Read Temperature on LEDS.\n",
                              read_temp_cb);

    debug_remote_cmd_register("leds", "",
                              "Read and decode status of led controllers.\n",
                              get_controller_status);

    debug_remote_cmd_register("lede", "",
                              "Move led controller into active mode.\n",
                              enable_controllers);  

    debug_remote_cmd_register("setdin", "[target] [output]",
                              "Set the satus of the DIN\n",
                              setdin_cb);  

    debug_remote_cmd_register("pulse", "",
                              "Pulse the LEDS\n",
                              start_pulse_cb);              
}
