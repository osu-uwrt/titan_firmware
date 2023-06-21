#include "hardware/dma.h"
#include "pico/time.h"
#include "pico/sync.h"

#include "driver/status_strip.h"
#include "ws2812.pio.h"

#define LED_UPDATE_INTERVAL_MS 50
#define LED_TIMER_PERIOD_TICKS 120   // Note this should be less than 256 to avoid multiplication overflows and divisible by 2
#define LED_FAST_FLASH_PERIOD 6      // This should be divisible by 2 and LED_TIMER_PERIOD_TICKS divisible by this
#define LED_SLOW_FLASH_PERIOD 40     // This should be divisible by 2 and LED_TIMER_PERIOD_TICKS divisible by this
// Note breath uses LED_TIMER_PERIOD

#define LED_FLASH_PULSE_PERIOD 6     // Note this should be less than 256 to avoid multiplication overflows
#define LED_FLASH_PULSE_COUNT 2

#define NUM_LEDS 12
static_assert(NUM_LEDS % 4 == 0, "Number of LEDs must be divisible by 4");

// Startup flashing configuation
#define STARTUP_TICK_DURATION 2 // Number of LED updates between each startup tick
#define STARTUP_COLOR_CODE (  \
    (10 << 24)  | /* Green */ \
    (255 << 16) | /* Red */   \
    (0 << 8)    ) /* Blue */

enum startup_phase {
    STARTUP_PHASE_GROW = 0,
    STARTUP_PHASE_FULL,
    STARTUP_PHASE_SHRINK,
    STARTUP_PHASE_RESET_DMA,
    STARTUP_PHASE_DONE
};

union ws2812_command {
    uint32_t data;
    struct __attribute__((__packed__)) {
        uint8_t reserved;
        uint8_t blue;
        uint8_t red;
        uint8_t green;
    } cmd;
};

static struct status_strip_inst {
    // Configuration
    PIO pio;
    uint sm;
    uint offset;
    bool first_pixel_is_rear;

    // DMA config
    uint dma_ctrl_chan;
    uint dma_data_chan;
    union ws2812_command end_cmd;
    union ws2812_command middle_cmd;
    struct {uint32_t len; const uint32_t *data;} control_blocks[4];  // 3 commands: start, middle, and end, and 1 for stop
    repeating_timer_t refresh_timer;

    // LED state config
    volatile bool enabled;
    enum status_strip_mode mode;
    uint8_t red_target;
    uint8_t green_target;
    uint8_t blue_target;
    uint timer;

    // Flash State Config
    volatile bool flash_active; // Voltaile as this is used to protect the flashing state, rather than disabling interrupts
    uint flash_timer;
    uint flash_count;
    uint8_t red_flash_target;
    uint8_t green_flash_target;
    uint8_t blue_flash_target;

    // Startup Flash
    enum startup_phase startup_flash_phase;
    uint startup_flash_index;
} local_inst;
static struct status_strip_inst * const inst = &local_inst;

bool __time_critical_func(status_strip_refresh)(__unused repeating_timer_t *rt) {
    // Note: No need to check if DMA transfer still active
    // This is because the repeating timer should always be much larger than transmission time
    // and as this is a transmission, there is no way for the PIO to block for extended periods of time
    // The only requirement is that LED_UPDATE_INTERVAL_MS is large enough to avoid overwhelming the PIO

    // Recompute data to transmit depending on timer value

    // Handle startup flash
    if (inst->startup_flash_phase < STARTUP_PHASE_DONE) {
        inst->timer++;
        if (inst->timer >= STARTUP_TICK_DURATION) {
            inst->timer = 0;
            // Grow and shrink are the most complex
            if (inst->startup_flash_phase == STARTUP_PHASE_GROW || inst->startup_flash_phase == STARTUP_PHASE_SHRINK) {
                inst->startup_flash_index++;

                // Compute the color
                // We want to color the middle if
                // a) We are at the start, meaning the rear should be colored, so the first pixel should be in the front
                // b) We are at the second phase, meaning the front should be colored, so the first pixel should be in the rear
                if ((inst->startup_flash_phase == 0) ^ inst->first_pixel_is_rear) {
                    inst->middle_cmd.data = STARTUP_COLOR_CODE;
                    inst->end_cmd.data = 0;
                } else {
                    inst->middle_cmd.data = 0;
                    inst->end_cmd.data = STARTUP_COLOR_CODE;
                }

                // Startup
                // Index should be number of pixels in the rear
                // When we're in phase 0, these pixels will be lit
                // When we're in phase 1, the pixels will be dark
                if (inst->first_pixel_is_rear) {
                    // So the end counts are the number of pixels in the rear lit up
                    inst->control_blocks[0].len = inst->startup_flash_index;
                    inst->control_blocks[1].len = NUM_LEDS - (inst->startup_flash_index * 2);
                    inst->control_blocks[2].len = inst->startup_flash_index;
                }
                else {
                    // So the middle count is the number of pixels in the rear * 2
                    inst->control_blocks[0].len = (NUM_LEDS/2) - inst->startup_flash_index;
                    inst->control_blocks[1].len = inst->startup_flash_index * 2;
                    inst->control_blocks[2].len = (NUM_LEDS/2) - inst->startup_flash_index;
                }

                // Handle switching to the next state
                if (inst->startup_flash_index == (NUM_LEDS/2)-1) {
                    inst->startup_flash_index = 0;
                    inst->startup_flash_phase++;
                }
            }
            else if (inst->startup_flash_phase == STARTUP_PHASE_FULL) {
                // Set strip to all-on with startup color
                inst->control_blocks[0].len = NUM_LEDS / 4;
                inst->control_blocks[1].len = NUM_LEDS / 2;
                inst->control_blocks[2].len = NUM_LEDS / 4;
                inst->middle_cmd.data = STARTUP_COLOR_CODE;
                inst->end_cmd.data = STARTUP_COLOR_CODE;

                inst->startup_flash_phase++;
            }
            else if (inst->startup_flash_phase == STARTUP_PHASE_RESET_DMA) {
                inst->control_blocks[0].len = NUM_LEDS / 4;
                inst->control_blocks[1].len = NUM_LEDS / 2;
                inst->control_blocks[2].len = NUM_LEDS / 4;
                inst->middle_cmd.data = 0;
                inst->end_cmd.data = 0;
                inst->startup_flash_phase++;
                // Note this won't clear for the entire brightness
                // Since this releases to normal operation, the timer will default to its normal operation
                // So it will only be blank for 1 timer cycle
            }
        }
    }

    // Everything else is normal operation
    else {
        union ws2812_command command;
        if (inst->mode == STATUS_STRIP_MODE_SOLID) {
            command.cmd.red = inst->red_target;
            command.cmd.green = inst->green_target;
            command.cmd.blue = inst->blue_target;
        }
        else if (inst->mode == STATUS_STRIP_MODE_FAST_FLASH || inst->mode == STATUS_STRIP_MODE_SLOW_FLASH) {
            uint flash_period = (inst->mode == STATUS_STRIP_MODE_FAST_FLASH ? LED_FAST_FLASH_PERIOD : LED_SLOW_FLASH_PERIOD);
            if (inst->timer % flash_period < (flash_period / 2)) {
                command.cmd.red = inst->red_target;
                command.cmd.green = inst->green_target;
                command.cmd.blue = inst->blue_target;
            }
            else {
                // Set blank
                command.data = 0;
            }
        }
        else if (inst->mode == STATUS_STRIP_MODE_BREATH) {
            if (inst->timer % LED_TIMER_PERIOD_TICKS < (LED_TIMER_PERIOD_TICKS / 2)) {
                // Handle rising fade
                command.cmd.red = (((uint32_t)inst->red_target) * inst->timer) / (LED_TIMER_PERIOD_TICKS / 2);
                command.cmd.green = (((uint32_t)inst->green_target) * inst->timer) / (LED_TIMER_PERIOD_TICKS / 2);
                command.cmd.blue = (((uint32_t)inst->blue_target) * inst->timer) / (LED_TIMER_PERIOD_TICKS / 2);
            }
            else {
                // Handle falling fade
                command.cmd.red = (inst->red_target) - (((uint32_t)inst->red_target) * (inst->timer - (LED_TIMER_PERIOD_TICKS / 2)) / (LED_TIMER_PERIOD_TICKS / 2));
                command.cmd.green = (inst->green_target) - (((uint32_t)inst->green_target) * (inst->timer - (LED_TIMER_PERIOD_TICKS / 2)) / (LED_TIMER_PERIOD_TICKS / 2));
                command.cmd.blue = (inst->blue_target) - (((uint32_t)inst->blue_target) * (inst->timer - (LED_TIMER_PERIOD_TICKS / 2)) / (LED_TIMER_PERIOD_TICKS / 2));
            }
            // Square it to make fading look smoother
            command.cmd.red = (((uint32_t) command.cmd.red) * ((uint32_t) command.cmd.red)) >> 8;
            command.cmd.green = (((uint32_t) command.cmd.green) * ((uint32_t) command.cmd.green)) >> 8;
            command.cmd.blue = (((uint32_t) command.cmd.blue) * ((uint32_t) command.cmd.blue)) >> 8;
        }
        else {
            command.data = 0;
        }
        inst->timer = (inst->timer + 1) % LED_TIMER_PERIOD_TICKS;   // Tick the timer

        // Handle quick flash requests
        if (inst->flash_active) {
            // First compute color for this round of flashing
            union ws2812_command flash_command;

            if (inst->flash_timer % LED_FLASH_PULSE_PERIOD < (LED_FLASH_PULSE_PERIOD / 2)) {
                // Handle rising fade
                flash_command.cmd.red = (((uint32_t)inst->red_flash_target) * inst->flash_timer) / (LED_FLASH_PULSE_PERIOD / 2);
                flash_command.cmd.green = (((uint32_t)inst->green_flash_target) * inst->flash_timer) / (LED_FLASH_PULSE_PERIOD / 2);
                flash_command.cmd.blue = (((uint32_t)inst->blue_flash_target) * inst->flash_timer) / (LED_FLASH_PULSE_PERIOD / 2);
            }
            else {
                // Handle falling fade
                flash_command.cmd.red = (inst->red_flash_target) - (((uint32_t)inst->red_flash_target) * (inst->flash_timer - (LED_FLASH_PULSE_PERIOD / 2)) / (LED_FLASH_PULSE_PERIOD / 2));
                flash_command.cmd.green = (inst->green_flash_target) - (((uint32_t)inst->green_flash_target) * (inst->flash_timer - (LED_FLASH_PULSE_PERIOD / 2)) / (LED_FLASH_PULSE_PERIOD / 2));
                flash_command.cmd.blue = (inst->blue_flash_target) - (((uint32_t)inst->blue_flash_target) * (inst->flash_timer - (LED_FLASH_PULSE_PERIOD / 2)) / (LED_FLASH_PULSE_PERIOD / 2));
            }

            // Then compute the next timer and count values
            inst->flash_timer++;
            if (inst->flash_timer >= LED_FLASH_PULSE_PERIOD) {
                inst->flash_timer = 0;
                inst->flash_count++;
                if (inst->flash_count >= LED_FLASH_PULSE_COUNT) {
                    inst->flash_active = false;
                }
            }

            // Finally update the rear set of LEDs
            if (inst->first_pixel_is_rear) {
                inst->middle_cmd.data = command.data;
                inst->end_cmd.data = flash_command.data;
            }
            else {
                inst->end_cmd.data = command.data;
                inst->middle_cmd.data = flash_command.data;
            }
        }
        else {
            inst->end_cmd.data = command.data;
            inst->middle_cmd.data = command.data;
        }
    }

    // Clear color output if LEDs are disabled
    if (!inst->enabled) {
        inst->end_cmd.data = 0;
        inst->middle_cmd.data = 0;
    }

    // Transmit data
    dma_channel_set_read_addr(inst->dma_ctrl_chan, &inst->control_blocks[0], true);

    return true;
}

void status_strip_init(PIO pio, uint sm, uint pin, bool first_pixel_is_rear) {
    inst->pio = pio;
    inst->sm = sm;
    inst->first_pixel_is_rear = first_pixel_is_rear;

    // Configure PIO
    inst->offset = pio_add_program(pio, &ws2812_program);
    pio_sm_claim(pio, sm);
    ws2812_program_init(pio, sm, inst->offset, pin, 800000, false);

    // Configure DMA
    inst->dma_ctrl_chan = dma_claim_unused_channel(true);
    inst->dma_data_chan = dma_claim_unused_channel(true);

    // Configure control DMA channel
    // Control blocks determine the source address and length of transfer
    // Used to allow sending large several repeated bytes by only updating 2 ints
    inst->control_blocks[0].data = &inst->end_cmd.data;
    inst->control_blocks[0].len = NUM_LEDS / 4;     // First quarter of strip to end command (half of the start)
    inst->control_blocks[1].data = &inst->middle_cmd.data;
    inst->control_blocks[1].len = NUM_LEDS / 2;     // Middle of strip is the middle command (center half of strip)
    inst->control_blocks[2].data = &inst->end_cmd.data;
    inst->control_blocks[2].len = NUM_LEDS / 4;     // Last quarter of strip to end command (half of the end)
    inst->control_blocks[3].data = NULL;            // Send NULL to terminate transfer
    inst->control_blocks[3].len = 0;

    dma_channel_config c = dma_channel_get_default_config(inst->dma_ctrl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, 3); // 1 << 3 byte boundary on write ptr
    dma_channel_configure(
        inst->dma_ctrl_chan,
        &c,
        &dma_hw->ch[inst->dma_data_chan].al3_transfer_count, // Initial write address
        &inst->control_blocks[0],                            // Initial read address
        2,                                                   // Halt after each control block
        false                                                // Don't start yet
    );

    // Configure the data DMA channel which will actually perform data transfer
    // This is being configured at the end of each transfer by the control DMA
    c = dma_channel_get_default_config(inst->dma_data_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    // Trigger ctrl_chan when data_chan completes
    channel_config_set_chain_to(&c, inst->dma_ctrl_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    // Raise the IRQ flag when 0 is written to a trigger register (end of chain):
    channel_config_set_irq_quiet(&c, true);
    dma_channel_configure(
        inst->dma_data_chan,
        &c,
        &pio->txf[sm],
        NULL,           // Initial read address and transfer count are unimportant;
        0,              // the control channel will reprogram them each time.
        false           // Don't start yet.
    );

    // Set up the startup flash
    inst->startup_flash_phase = STARTUP_PHASE_GROW;
    inst->startup_flash_index = 0;

    // Start with LEDs enabled
    inst->enabled = true;

    // Schedule the transmit
    inst->timer = 0;
    inst->flash_active = false;
    hard_assert(add_repeating_timer_ms(LED_UPDATE_INTERVAL_MS, status_strip_refresh, NULL, &inst->refresh_timer));
}

void status_strip_set(enum status_strip_mode mode, uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t prev_interrupts = save_and_disable_interrupts();
    inst->timer = 0;
    inst->mode = mode;
    inst->red_target = red;
    inst->green_target = green;
    inst->blue_target = blue;
    restore_interrupts(prev_interrupts);
}

void status_strip_flash_front(uint8_t red, uint8_t green, uint8_t blue) {
    inst->flash_active = false;
    inst->red_flash_target = red;
    inst->green_flash_target = green;
    inst->blue_flash_target = blue;
    inst->flash_timer = 0;
    inst->flash_count = 0;
    inst->flash_active = true;
}

void status_strip_enable(void) {
    inst->enabled = true;
}

void status_strip_disable(void) {
    inst->enabled = false;
}
