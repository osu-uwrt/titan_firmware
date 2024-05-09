#include "display.h"

#include "core1.h"
#include "uwrt_logo.h"

#include "driver/ssd1306.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdbool.h>
#include <stdio.h>

// How often to redraw the screen when no activity occurs
#define DISPLAY_UPDATE_INTERVAL_MS 300
// How long to show splash screen on startup
#define DISPLAY_SPLASH_TIME_MS 1000
// How long to show an interactive screen
#define DISPLAY_HOLD_MS 15000
// How long before the display fades out that the contrast should be dimmed
#define DISPLAY_DIM_TIME_MS 5000
// The contrast of the display when in the dim state
#define DISPLAY_DIM_CONTRAST 0x10
// How long to show a message on the screen
#define DISPLAY_MESSAGE_TIME_MS 2000
// Need to hold the reed switch for 2 seconds to register as a switch hold
#define DISPLAY_SWITCH_HOLD_TIME_MS 2000

// ========================================
// Menu/Screen Definitions
// ========================================

// Allows abstraction of button/event code from the various display logic code
// so it's easier to add new items to the screen, at the limitation of the screens
// lacking interaction (except for the menu)

// The assorted screens, menus, and actions that can be shown
// These are the valid operations that a menu entry can perform
enum menu_operation {
    OP_STARTUP,
    OP_SHOW_MESSAGE,
    OP_SHOW_SPLASH,
    OP_SHOW_SCREEN,
    OP_SHOW_MENU,
    OP_DO_ACTION,
    OP_DISPLAY_OFF
};
// These are the valid menus that can be displayed
enum menu_type { MENU_MAIN, MENU_MORE };
// These are the valid screens that can be displayed
enum screen_type { SCREEN_SOC, SCREEN_CURRENT, SCREEN_PACK_STATUS, SCREEN_LIFE_STATS, SCREEN_CELL_VOLTAGES };
// These are the valid actions that can be executed on the battery
enum action_type { ACTION_POWER_CYCLE };

typedef struct display_target {
    enum menu_operation op;
    union {
        enum menu_type menu_target;
        enum screen_type screen_target;
        enum action_type action_target;
    } op_data;
} display_target_t;

typedef struct menu_entry {
    const char *msg;
    display_target_t target;
} menu_entry_t;

#define NUM_MENU_ENTRIES 4
typedef menu_entry_t menu_definition_arr[NUM_MENU_ENTRIES];

const menu_definition_arr menu_def_main = {
    { .msg = "1. Relative SOC", .target = { .op = OP_SHOW_SCREEN, .op_data = { .screen_target = SCREEN_SOC } } },
    { .msg = "2. Current/Voltage", .target = { .op = OP_SHOW_SCREEN, .op_data = { .screen_target = SCREEN_CURRENT } } },
    { .msg = "3. Pack Status", .target = { .op = OP_SHOW_SCREEN, .op_data = { .screen_target = SCREEN_PACK_STATUS } } },
    { .msg = "4. More...", .target = { .op = OP_SHOW_MENU, .op_data = { .menu_target = MENU_MORE } } }
};

const menu_definition_arr menu_def_more = {
    { .msg = "Detailed Stats", .target = { .op = OP_SHOW_SCREEN, .op_data = { .screen_target = SCREEN_LIFE_STATS } } },
    { .msg = "Cell Voltages",
      .target = { .op = OP_SHOW_SCREEN, .op_data = { .screen_target = SCREEN_CELL_VOLTAGES } } },
    { .msg = "Power Cycle Robot",
      .target = { .op = OP_DO_ACTION, .op_data = { .action_target = ACTION_POWER_CYCLE } } },
    { .msg = "Back <-", .target = { .op = OP_SHOW_MENU, .op_data = { .menu_target = MENU_MAIN } } }
};

const menu_entry_t *const menu_definitions[] = { [MENU_MAIN] = menu_def_main, [MENU_MORE] = menu_def_more };

// ========================================
// Screen State Storage
// ========================================

struct display_state {
    display_target_t cur_target;       // The target state for the display to go to
    absolute_time_t next_state_time;   // Timeout to tell the state machine to advance to its next default state (should
                                       // all lead to display off)
    absolute_time_t next_redraw;       // Time that the display will re-run its redraw function automatically
    absolute_time_t btn_hold_timeout;  // Time when the button is considered to have been held in this state
    bool last_switch_state;            // State of the button when it was last held down
    uint8_t selected_idx;              // Index for the currently selected menu button
    bool msg_pending;                  // Set to true by draw message function before calling tick, to enter msg state
    uint8_t last_contrast;             // The last display contrast value
};

static struct display_state state = {};

static void display_show_menu(const menu_definition_arr menu_def, uint selected_idx, uint circle_fill_deg) {
    // Clear screen and draw bar for the selected menu entry
    ssd1306_Fill(Black);
    ssd1306_FillRectangle(0, selected_idx * 8, SSD1306_WIDTH - 1, (selected_idx + 1) * 8 - 1, White);

    // Progress circle indicate the presence of input
    if (circle_fill_deg > 30) {
        ssd1306_DrawArcWithRadiusLine(122, 4 + (8 * selected_idx), 5, 0, circle_fill_deg, Black);
    }

    // Render the menu entry text
    for (uint i = 0; i < NUM_MENU_ENTRIES; i++) {
        ssd1306_SetCursor(0, 8 * i);
        SSD1306_COLOR color = (selected_idx == i) ? Black : White;
        ssd1306_WriteString(menu_def[i].msg, Font_6x8, color);
    }

    ssd1306_UpdateScreen();
}

static void display_format_remaining_time(char *buf, size_t buf_len, uint16_t time_min) {
    if (time_min == 65535) {
        snprintf(buf, buf_len, "N/A");
        buf[buf_len - 1] = 0;
    }
    else if (time_min > 10 * 60) {
        // Over 10 hours, just print hours rounded to ones place
        snprintf(buf, buf_len, "%d h", time_min / 60);
    }
    else if (time_min > 60) {
        snprintf(buf, buf_len, "%d.%d h", time_min / 60, (time_min % 60) / 6);
    }
    else {
        snprintf(buf, buf_len, "%d m", time_min);
    }
}

static void display_show_splash_screen(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Ohio State", Font_7x10, White);
    ssd1306_SetCursor(0, 11);
    ssd1306_WriteString("Underwater", Font_7x10, White);
    ssd1306_SetCursor(0, 22);
    ssd1306_WriteString("Robotics", Font_7x10, White);
    ssd1306_DrawBitmap(79, 0, uwrt_logo_bin, uwrt_logo_bin_width, uwrt_logo_bin_height, White);
    ssd1306_UpdateScreen();
}

static void display_show_main_screen(enum screen_type selected_screen) {
    bq_mfg_info_t mfg_info;
    core1_get_pack_mfg_info(&mfg_info);

    char buf[16];
    ssd1306_Fill(Black);

    // Draw Pack Year/Set
    ssd1306_SetCursor(0, 0);
    uint8_t packYearSubseries = (mfg_info.serial % 2);
    if (packYearSubseries == 0)
        packYearSubseries = 2;
    snprintf(buf, sizeof(buf), "ID: %d-%d", mfg_info.mfg_year, packYearSubseries);
    ssd1306_WriteString(buf, Font_6x8, White);

    // Draw Pack Serial Number
    ssd1306_SetCursor(120, 2);
    snprintf(buf, sizeof(buf), "%d", mfg_info.serial);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_DrawRectangle(118, 0, 127, 11, White);

    // Draw large filled box for SOC/current screens
    if (selected_screen == SCREEN_SOC || selected_screen == SCREEN_CURRENT) {
        ssd1306_FillRectangle(0, 11, 79, 32, White);
        ssd1306_SetCursor(2, 13);
        if (selected_screen == SCREEN_SOC)
            snprintf(buf, sizeof(buf), "SOC %d%%", core1_soc());
        else if (selected_screen == SCREEN_CURRENT)
            snprintf(buf, sizeof(buf), "%.2f A", (float) core1_current() / 1000.0);
        ssd1306_WriteString(buf, Font_11x18, Black);
    }

    // Draw bottom right details section
    if (selected_screen == SCREEN_SOC) {
        bool dsg_mode;
        uint16_t time_remaining = core1_time_remaining(&dsg_mode);

        if (time_remaining != 65535) {
            // If we have a valid time reamining, render it

            ssd1306_SetCursor(86, 12);
            if (dsg_mode) {
                snprintf(buf, sizeof(buf), "ETtE:");  // Estimated time to empty
            }
            else {
                snprintf(buf, sizeof(buf), "ETtF:");  // Estimated time to full
            }
            ssd1306_WriteString(buf, Font_7x10, White);

            ssd1306_SetCursor(86, 22);
            if (dsg_mode) {
                display_format_remaining_time(buf, sizeof(buf), time_remaining);
            }
            else {
                display_format_remaining_time(buf, sizeof(buf), time_remaining);
            }
            ssd1306_WriteString(buf, Font_7x10, White);
        }
        else {
            // Don't show invalid time remaining, just show the voltage
            ssd1306_SetCursor(86, 22);
            snprintf(buf, sizeof(buf), "%.2fV", (float) core1_voltage() / 1000.0);
            ssd1306_WriteString(buf, Font_7x10, White);
        }
    }
    else if (selected_screen == SCREEN_CURRENT) {
        // Show the voltage in auxiliary slot
        ssd1306_SetCursor(86, 22);
        snprintf(buf, sizeof(buf), "%.2fV", (float) core1_voltage() / 1000.0);
        ssd1306_WriteString(buf, Font_7x10, White);
    }

    ssd1306_UpdateScreen();
}

void display_init(void) {
    ssd1306_Init();

    // Initialize state variable
    // Need to get the current switch state on init so we know the state on initial startup
    state.last_switch_state = gpio_get(SWITCH_SIGNAL_PIN);

    // Tick the display once to show the splash screen
    display_tick();
}

void display_tick() {
    display_target_t target_state = state.cur_target;
    uint8_t new_selected_idx = state.selected_idx;

    // Stage 1: Check current state

    // Compute if the display should be dimmed (if we stay on the current state)
    bool display_should_dim = false;
    switch (state.cur_target.op) {
    case OP_SHOW_MENU:
    case OP_SHOW_SCREEN:
        if (absolute_time_diff_us(delayed_by_ms(get_absolute_time(), DISPLAY_DIM_TIME_MS), state.next_state_time) < 0) {
            display_should_dim = true;
        }
        break;
    // All other states don't have dimming
    default:
        break;
    }

    // Compute Button Hold/Tap Events
    bool cur_switch_state = gpio_get(SWITCH_SIGNAL_PIN);

    bool switch_tap_fired = false;   // True if the switch was tapped
    bool switch_hold_fired = false;  // True if the switch was held
    if (cur_switch_state != state.last_switch_state) {
        bool was_hold = time_reached(state.btn_hold_timeout);
        if (!cur_switch_state && !was_hold) {
            switch_tap_fired = true;
        }
        state.btn_hold_timeout = make_timeout_time_ms(DISPLAY_SWITCH_HOLD_TIME_MS);
    }
    if (!is_nil_time(state.btn_hold_timeout) && time_reached(state.btn_hold_timeout)) {
        // The button was held down, set the event
        if (cur_switch_state) {
            switch_hold_fired = true;
        }
        // Set hold timeout to nil time to make sure we don't fire the hold event again until the button gets held down
        state.btn_hold_timeout = nil_time;
    }
    state.last_switch_state = cur_switch_state;

    // If the switch is tapped in interactive states, clear the display hold timeout
    // This allows the screen to be kept alive by the user tapping the button
    if (switch_tap_fired) {
        switch (state.cur_target.op) {
        case OP_SHOW_SCREEN:
        case OP_SHOW_MENU:
            state.next_state_time = make_timeout_time_ms(DISPLAY_HOLD_MS);
            break;
        default:
            break;
        }
    }

    // Stage 2: Determine the next target state based on button presses and timers
    // This will write target_state depending on previous state and button state

    // Timeout state transition
    // This controls directs the default flow of the screen if no action is taken by button presses
    if (time_reached(state.next_state_time)) {
        switch (state.cur_target.op) {
        // Show splash screen out of startup
        case OP_STARTUP:
            target_state.op = OP_SHOW_SPLASH;
            break;
        // After a message or splash screen
        case OP_SHOW_SPLASH:
        case OP_SHOW_MESSAGE:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_SOC;
            break;
        case OP_SHOW_SCREEN:
        case OP_SHOW_MENU:
        case OP_DO_ACTION:
        default:
            target_state.op = OP_DISPLAY_OFF;
            break;
        }
    }

    // Handle taps/fires to switch state
    if (switch_tap_fired) {
        switch (state.cur_target.op) {
        case OP_SHOW_MENU:
            new_selected_idx += 1;
            new_selected_idx %= NUM_MENU_ENTRIES;
            break;
        case OP_SHOW_SCREEN:
            // If display is dimmed, just let the code in stage 1 brighten the screen
            // But, if it's full brightness, show the menu
            if (!display_should_dim) {
                target_state.op = OP_SHOW_MENU;
                target_state.op_data.menu_target = MENU_MAIN;
            }
            else {
                display_should_dim = false;
            }
            break;
        case OP_DISPLAY_OFF:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_SOC;
            break;
        default:
            break;
        }
    }

    if (switch_hold_fired) {
        switch (state.cur_target.op) {
        case OP_SHOW_MENU:
            target_state = menu_definitions[state.cur_target.op_data.menu_target][state.selected_idx].target;
            break;
        case OP_DISPLAY_OFF:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_SOC;
            break;
        default:
            break;
        }
    }

    // Message takes highest priority, if we have a pending message force enter show message state
    if (state.msg_pending) {
        target_state.op = OP_SHOW_MESSAGE;
    }

    // Stage 3: Interpret the meaning out of the various state transitions and timeouts to prepare for drawing
    // Do not write target_state after this point

    bool state_transitioned = (target_state.op != state.cur_target.op);
    if (!state_transitioned) {
        // Compute state transitioned for union memebers if the target state matches
        switch (target_state.op) {
        case OP_SHOW_SCREEN:
            state_transitioned = (target_state.op_data.screen_target != state.cur_target.op_data.screen_target);
            break;
        case OP_SHOW_MENU:
            state_transitioned = (target_state.op_data.menu_target != state.cur_target.op_data.menu_target);
            break;
        case OP_DO_ACTION:
            state_transitioned = (target_state.op_data.action_target != state.cur_target.op_data.action_target);
            break;
        default:
            break;
        }
    }

    bool redraw_needed =
        time_reached(state.next_redraw) || state_transitioned || (new_selected_idx != state.selected_idx);

    // If the display is idle, dim the display after inactive for a while
    uint8_t target_contrast = target_state.op == OP_DISPLAY_OFF ? 0 : 0xFF;
    if (!state_transitioned && display_should_dim) {
        target_contrast = DISPLAY_DIM_CONTRAST;
    }

    // Stage 4: Update the display according to the new display state
    // Do not write target_state, state_transitioned or redraw_needed variables after this point

    // Need to send the display on command if we transitioned from display off
    if (state_transitioned && state.cur_target.op == OP_DISPLAY_OFF) {
        ssd1306_SetDisplayOn(1);
    }

    // Need to change the contrast if the current display contrast doesn't match what we expect
    if (state.last_contrast != target_contrast) {
        ssd1306_SetContrast(target_contrast);
        state.last_contrast = target_contrast;
    }

    if (redraw_needed) {
        switch (target_state.op) {
        case OP_SHOW_SPLASH:
            if (state_transitioned) {
                // Show splash screen for 1 second
                state.next_state_time = make_timeout_time_ms(DISPLAY_SPLASH_TIME_MS);
                display_show_splash_screen();
            }
            // No need to redraw the splash screen, nothing is changing
            break;

        case OP_SHOW_MESSAGE:
            if (state_transitioned) {
                // Keep message on screen for this amount of time
                state.next_state_time = make_timeout_time_ms(DISPLAY_MESSAGE_TIME_MS);
            }
            // We don't redraw the message, since that is done by the message caller function
            break;

        case OP_SHOW_SCREEN:
            if (state_transitioned) {
                state.next_state_time = make_timeout_time_ms(DISPLAY_HOLD_MS);
            }
            switch (target_state.op_data.screen_target) {
            case SCREEN_SOC:
            case SCREEN_CURRENT:
            case SCREEN_PACK_STATUS:
                display_show_main_screen(target_state.op_data.screen_target);
                break;
            default:
                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString("Invalid Screen", Font_7x10, White);
                ssd1306_UpdateScreen();
                break;
            }
            break;

        case OP_SHOW_MENU:
            if (state_transitioned) {
                // Need to reset the menu index to the top
                new_selected_idx = 0;
                // Refresh state timeout if we changed index or we are on a new state
                state.next_state_time = make_timeout_time_ms(DISPLAY_HOLD_MS);
            }

            // Compute hold circle angle
            uint16_t sweep_angle = 0;
            if (cur_switch_state && !is_nil_time(state.btn_hold_timeout)) {
                int64_t hold_time_left_us = absolute_time_diff_us(get_absolute_time(), state.btn_hold_timeout);
                int64_t hold_time_ms = DISPLAY_SWITCH_HOLD_TIME_MS - (hold_time_left_us / 1000);
                if (hold_time_ms < 0) {
                    sweep_angle = 0;
                }
                else {
                    int64_t sweep_angle_long = (hold_time_ms * 360) / DISPLAY_SWITCH_HOLD_TIME_MS;
                    if (sweep_angle_long > 360) {
                        sweep_angle_long = 360;
                    }
                    else if (sweep_angle_long < 0) {
                        sweep_angle_long = 0;
                    }
                    sweep_angle = (uint16_t) sweep_angle_long;
                }
            }

            // Draw the menu
            display_show_menu(menu_definitions[target_state.op_data.menu_target], new_selected_idx, sweep_angle);
            break;

        case OP_DO_ACTION:
            if (state_transitioned) {
                state.next_state_time = make_timeout_time_ms(DISPLAY_HOLD_MS);
            }
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString("Not Yet Implemented", Font_7x10, White);
            ssd1306_UpdateScreen();
            break;

        case OP_DISPLAY_OFF:
        default:
            if (state_transitioned) {
                // We never leave the display off state without user input
                state.next_state_time = at_the_end_of_time;

                // We just got to this state, clear the display and turn it off
                ssd1306_Fill(Black);
                ssd1306_UpdateScreen();
                ssd1306_SetDisplayOn(0);
            }
            // Nothing to do to redraw an already off screen
            break;
        }
        state.next_redraw = make_timeout_time_ms(DISPLAY_UPDATE_INTERVAL_MS);
    }

    // Stage 5: Update saved state to reflect the new state
    state.cur_target = target_state;
    state.selected_idx = new_selected_idx;
}

void display_show_msg(const char *msg, const char *submsg) {
    ssd1306_Fill(Black);
    if (msg && *msg) {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(msg, Font_11x18, White);
    }
    if (submsg && *submsg) {
        ssd1306_SetCursor(0, 24);
        ssd1306_WriteString(submsg, Font_6x8, White);
    }
    ssd1306_UpdateScreen();

    state.msg_pending = true;
    display_tick();
    state.msg_pending = false;
}

bool display_check_on(void) {
    return state.cur_target.op != OP_DISPLAY_OFF;
}
