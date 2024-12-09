#include "display.h"

#include "core1.h"
#include "safety_interface.h"
#include "uwrt_logo.h"

#include "driver/canbus.h"
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
enum screen_type {
    SCREEN_SOC,
    SCREEN_CURRENT,
    SCREEN_PACK_STATUS,
    SCREEN_LIFE_STATS,  // TODO: Implement screen
    SCREEN_CELL_VOLTAGES,

    // Static screens
    SCREEN_BQ_MISSING,
    SCREEN_BQ_INITIALIZING,
    SCREEN_BQ_PERM_FAIL,
    SCREEN_BQ_SHUTDOWN,
    SCREEN_BQ_PWR_CYCLE,
    SCREEN_BQ_XDSG
};
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

static const char *const protection_names[] = {
    [0] = "Cell Undervolt",  [1] = "Cell Overvolt",   [2] = "Overcur. Chg1",
    [3] = "Overcur. Chg2",   [4] = "Overcur. Dsg1",   [5] = "Overcur. Dsg2",
    [6] = "Overld. Dsg",     [7] = "OvrldDsg Latch",  [8] = "ASCC",
    [9] = "ASCCL",           [10] = "ASCD",           [11] = "ASCDL",
    [12] = "OverTemp Chg",   [13] = "OverTemp Dsg",   [14] = "CUVC",
    [16] = "OverTemp FET",   [18] = "PreChg Timeout", [20] = "Charge Timeout",
    [22] = "Overcharge",     [23] = "Overcharge Cur", [24] = "Overcharge Volt",
    [25] = "OverPreChg Cur", [26] = "UnderTemp Chg",  [27] = "UnderTemp Dsg",
    [28] = "OverVolt Latch", [29] = "OverCur. Dsg",
};

static const char *const pf_names[] = {
    [0] = "Cell Undervolt", [1] = "Cell Overvolt",  [2] = "Overcur. Chg",   [3] = "Overcur. Dsg",
    [4] = "OverTemp Cell",  [5] = "OverVolt Latch", [6] = "OverTemp FET",   [7] = "QMax Imbal",
    [8] = "Cell Bal Fail",  [9] = "Impedance Fail", [10] = "Capacity Deg.", [11] = "VoltImbal@Rest",
    [12] = "VoltImbal@Act", [13] = "Ovrld. Dsg",    [14] = "ASCCL",         [15] = "ASCDL",
    [16] = "Chg FET Fail",  [17] = "Dsg FET Fail",  [18] = "Overcur. Dsg",  [19] = "ChemFUSE Fail",
    [20] = "AFE Reg Fail",  [21] = "AFE Comm Fail", [22] = "2ndLvl Prot",   [23] = "PTC Fail",
    [24] = "Instr. Flash",  [26] = "Data Flash",    [28] = "Thermistor1",   [29] = "Thermistor2",
    [30] = "Thermistor3",   [31] = "Thermistor4",
};

static const char *const batt_state_names[] = {
    [BATT_STATE_UNINITIALIZED] = "Uninitialized",
    [BATT_STATE_DISCONNECTED] = "Disconnected",
    [BATT_STATE_REMOVED] = "Removed",
    [BATT_STATE_NEEDS_SHUTDOWN] = "Shutting Off",
    [BATT_STATE_INITIALIZING] = "Initializing",
    [BATT_STATE_XDSG] = "Dsg Inhibited",
    [BATT_STATE_DISCHARGING] = "Discharging",
    [BATT_STATE_CHARGING] = "Charging",
    [BATT_STATE_PERMENANT_FAIL] = "Permanent Fail",
    [BATT_STATE_POWER_CYCLE] = "Power Cycling",
    [BATT_STATE_LATCH_OFF] = "Latched Off",
};
static const size_t batt_state_name_count = (sizeof(batt_state_names) / sizeof(*batt_state_names));

// ========================================
// Screen State Storage
// ========================================

struct display_state {
    batt_state_t last_batt_state;      // The last battery state when display_tick was called
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

enum display_msg_icon { MSG_ICON_NONE, MSG_ICON_INFO, MSG_ICON_ALERT };

static void display_draw_message(enum display_msg_icon icon, const char *msg, const char *submsg) {
    ssd1306_Fill(Black);
    switch (icon) {
    case MSG_ICON_INFO:
        ssd1306_DrawBitmap(4, 4, info_large_bin, info_large_bin_width, info_large_bin_height, White);
        break;
    case MSG_ICON_ALERT:
        ssd1306_DrawBitmap(3, 3, alert_large_bin, alert_large_bin_width, alert_large_bin_height, White);
        break;
    default:
        break;
    }
    if (msg && *msg) {
        ssd1306_SetCursor(38, 0);
        ssd1306_WriteString(msg, Font_11x18, White);
    }
    if (submsg && *submsg) {
        ssd1306_SetCursor(38, 23);
        ssd1306_WriteString(submsg, Font_6x8, White);
    }
    ssd1306_UpdateScreen();
}

static void display_show_menu(const menu_definition_arr menu_def, uint selected_idx, uint circle_fill_deg) {
    // Clear screen and draw bar for the selected menu entry
    ssd1306_Fill(Black);
    ssd1306_FillRectangle(0, selected_idx * 8, SSD1306_WIDTH - 1, (selected_idx + 1) * 8 - 1, White);

    // Progress circle indicate the presence of input
    if (circle_fill_deg > 30) {
        ssd1306_DrawArcWithRadiusLine(122, 4 + (8 * selected_idx), 4, 0, circle_fill_deg, Black);
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

static void display_show_cell_voltage_screen(void) {
    ssd1306_Fill(Black);

    char buf[16];
    uint16_t cell_voltages[5];
    core1_cell_voltages(cell_voltages);

    for (int i = 0; i < 5; i++) {
        // Set cursor for the correct position for this cell
        ssd1306_SetCursor(i > 2 ? 66 : 0, (i % 3) * 10);
        // Render this cell's voltage
        snprintf(buf, sizeof(buf), "C%d: %d.%dV", i + 1, cell_voltages[i] / 1000, cell_voltages[i] % 1000);
        ssd1306_WriteString(buf, Font_6x8, White);
    }

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

    // Draw status bar icons
    // Show online icon if we have micro ros agent online
    extern bool mfg_readout_okay;
    if (mfg_readout_okay) {
        ssd1306_DrawBitmap(85, 0, can_online_bin, can_online_bin_width, can_online_bin_height, White);
    }
    else {
        ssd1306_DrawBitmap(85, 0, can_offline_bin, can_offline_bin_width, can_offline_bin_height, White);
    }

    // Show blinking alert on battery safety alerts
    uint32_t alert_faults = (*fault_list_reg) & ((1 << FAULT_BQ40_SAFETY_STATUS) | (1 << FAULT_BQ40_PF_STATUS));
    static bool alert_blink = false;
    if (!alert_blink && alert_faults) {
        alert_blink = true;
        ssd1306_DrawBitmap(70, 0, alert_small_bin, alert_small_bin_width, alert_small_bin_height, White);
    }
    else {
        alert_blink = false;
    }

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
        bool chg_mode;
        uint16_t time_remaining = core1_time_remaining(&chg_mode);

        if (time_remaining != 65535) {
            // If we have a valid time reamining, render it

            ssd1306_SetCursor(86, 13);
            if (chg_mode) {
                snprintf(buf, sizeof(buf), "ETtF:");  // Estimated time to full
            }
            else {
                snprintf(buf, sizeof(buf), "ETtE:");  // Estimated time to empty
            }
            ssd1306_WriteString(buf, Font_6x8, White);

            ssd1306_SetCursor(86, 22);
            display_format_remaining_time(buf, sizeof(buf), time_remaining);
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
    else if (selected_screen == SCREEN_PACK_STATUS) {
        if ((*fault_list_reg) & (1 << FAULT_BQ40_PF_STATUS) || (*fault_list_reg) & (1 << FAULT_BQ40_SAFETY_STATUS)) {
            // Static variable so we can slowly rotate through all present faults
            static absolute_time_t nextFaultIncrement;
            static int targetFaultIdx = 0;

            const char *faultTitle;
            const char *const *faultNameMap;

            int displayedFault = -1;
            int curFault = 0;
            int faultBitNum = 0;
            uint32_t faultReg;

            if ((*fault_list_reg) & (1 << FAULT_BQ40_PF_STATUS)) {
                faultReg = safety_fault_data[FAULT_BQ40_PF_STATUS].extra_data;
                faultTitle = "Permanent Fail:";
                faultNameMap = pf_names;
            }
            else {
                faultReg = safety_fault_data[FAULT_BQ40_SAFETY_STATUS].extra_data;
                faultTitle = "Safety Fault:";
                faultNameMap = protection_names;
            }

            while (faultReg) {
                // Current check if current fault bit fet
                if (faultReg & 1) {
                    if (curFault == 0) {
                        // Always set to first so we can roll over
                        displayedFault = faultBitNum;
                    }
                    if (curFault == targetFaultIdx) {
                        // This is the fault to be displayed in the rotation
                        displayedFault = faultBitNum;
                        break;
                    }
                    curFault++;
                }

                faultBitNum++;
                faultReg >>= 1;
            }

            // If we didn't break out of the loop, then we must have rolled over
            // Show the fault
            if (curFault != targetFaultIdx)
                targetFaultIdx = 0;

            const char *faultName = "Unknown?";
            if (displayedFault >= 0 && faultNameMap[displayedFault]) {
                faultName = faultNameMap[displayedFault];
            }

            ssd1306_SetCursor(2, 13);
            ssd1306_WriteString(faultTitle, Font_6x8, White);
            ssd1306_SetCursor(2, 22);
            ssd1306_WriteString(faultName, Font_6x8, White);

            // Increment so next refresh we show it
            if (is_nil_time(nextFaultIncrement)) {
                nextFaultIncrement = make_timeout_time_ms(1000);
            }
            else if (time_reached(nextFaultIncrement)) {
                targetFaultIdx++;
                nextFaultIncrement = make_timeout_time_ms(1000);
            }
        }
        else {
            // If no faults, just show the global pack state

            const char *pack_state = "Unknown?";
            batt_state_t state = core1_get_batt_state();

            if (state < batt_state_name_count && batt_state_names[state]) {
                pack_state = batt_state_names[state];
            }

            ssd1306_SetCursor(2, 13);
            ssd1306_WriteString("Pack State:", Font_6x8, White);
            ssd1306_SetCursor(2, 22);
            ssd1306_WriteString(pack_state, Font_6x8, White);
        }

        // Render Temperature
        ssd1306_SetCursor(97, 15);
        ssd1306_WriteString("Temp:", Font_6x8, White);
        char temp_msg[7] = {};
        int16_t temp = core1_batt_temp() - 2731;  // Convert 0.1K to 0.1C
        int16_t temp_decimal = temp % 10;
        temp /= 10;
        if (temp < 0) {
            temp_decimal = 10 - temp_decimal;
        }
        int val = snprintf(temp_msg, sizeof(temp_msg), "%d.%dC", temp, temp_decimal);
        ssd1306_SetCursor((val == 5 ? 97 : val == 4 ? 103 : 91), 23);
        ssd1306_WriteString(temp_msg, Font_6x8, White);
    }

    ssd1306_UpdateScreen();
}

void display_init(void) {
    ssd1306_Init();

    // Initialize state variable
    // Need to get the current switch state on init so we know the state on initial startup
    state.last_switch_state = gpio_get(SWITCH_SIGNAL_PIN);

    // Tick the display once to show the splash screen
    state.last_batt_state = BATT_STATE_UNINITIALIZED + 1;  // Force screen to refresh
    display_tick(BATT_STATE_UNINITIALIZED);
}

void display_tick(batt_state_t battery_state) {
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
        // Set hold timeout to nil time to make sure we don't fire the hold event again until the button gets
        // held down
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

    // Compute if we need to switch the display state because the battery state changed
    bool show_default_screen_for_batt_state = (state.last_batt_state != battery_state);
    if (state.cur_target.op == OP_SHOW_SPLASH && !time_reached(state.next_state_time)) {
        // If we're showing the splash screen right now, let it stay on the screen for a bit
        show_default_screen_for_batt_state = false;
        battery_state = state.last_batt_state;
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
        // After splash screen or a message goes away, go to default state for the screen
        case OP_SHOW_SPLASH:
        case OP_SHOW_MESSAGE:
            show_default_screen_for_batt_state = true;
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
                // Set the menu depending on which screen we're showing
                switch (state.cur_target.op_data.screen_target) {
                case SCREEN_BQ_INITIALIZING:
                case SCREEN_BQ_MISSING:
                case SCREEN_BQ_PERM_FAIL:
                    // TODO: Enable debug menu
                    // right now just fall through
                case SCREEN_BQ_SHUTDOWN:
                case SCREEN_BQ_PWR_CYCLE:
                    // No debug menu access
                    break;
                case SCREEN_BQ_XDSG:
                    // XDSG alert will just show the normal menu when clicked
                case SCREEN_SOC:
                case SCREEN_CURRENT:
                case SCREEN_PACK_STATUS:
                    // Default menu
                    target_state.op = OP_SHOW_MENU;
                    target_state.op_data.menu_target = MENU_MAIN;
                    break;
                case SCREEN_LIFE_STATS:
                case SCREEN_CELL_VOLTAGES:
                    // Advanced menu
                    target_state.op = OP_SHOW_MENU;
                    target_state.op_data.menu_target = MENU_MORE;
                    break;
                }
            }
            else {
                display_should_dim = false;
            }
            break;
        case OP_DISPLAY_OFF:
            // If the display is off, and woken up, go to the default screen for that battery state
            show_default_screen_for_batt_state = true;
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
            // If the display is off, and woken up, go to the default screen for that battery state
            show_default_screen_for_batt_state = true;
            break;
        default:
            break;
        }
    }

    // Set the default screen for whenever the battery state changes
    if (show_default_screen_for_batt_state) {
        switch (battery_state) {
        case BATT_STATE_UNINITIALIZED:
            target_state.op = OP_SHOW_SPLASH;
            break;
        case BATT_STATE_DISCONNECTED:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_BQ_MISSING;
            break;
        case BATT_STATE_NEEDS_SHUTDOWN:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_BQ_SHUTDOWN;
            break;
        case BATT_STATE_INITIALIZING:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_BQ_INITIALIZING;
            break;
        case BATT_STATE_PERMENANT_FAIL:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_BQ_PERM_FAIL;
            break;
        case BATT_STATE_POWER_CYCLE:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_BQ_PWR_CYCLE;
            break;
        case BATT_STATE_XDSG:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target = SCREEN_BQ_XDSG;
            break;
        case BATT_STATE_LATCH_OFF:
            // TODO: To be implemented

        case BATT_STATE_REMOVED:
        case BATT_STATE_DISCHARGING:
        case BATT_STATE_CHARGING:
            target_state.op = OP_SHOW_SCREEN;
            target_state.op_data.screen_target =
                (*fault_list_reg & (1 << FAULT_BQ40_SAFETY_STATUS) ? SCREEN_PACK_STATUS : SCREEN_SOC);
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

            char err_msg[16];

            switch (target_state.op_data.screen_target) {
            case SCREEN_SOC:
            case SCREEN_CURRENT:
            case SCREEN_PACK_STATUS:
                display_show_main_screen(target_state.op_data.screen_target);
                break;

            case SCREEN_CELL_VOLTAGES:
                display_show_cell_voltage_screen();
                break;

            // Static screens
            case SCREEN_BQ_MISSING:
                snprintf(err_msg, sizeof(err_msg), "Err: 0x%08lX",
                         safety_fault_data[FAULT_BQ40_NOT_CONNECTED].extra_data);
                err_msg[sizeof(err_msg) - 1] = 0;
                display_draw_message(MSG_ICON_ALERT, "No BQ40", err_msg);
                break;
            case SCREEN_BQ_PERM_FAIL:
                snprintf(err_msg, sizeof(err_msg), "Err: 0x%08lX", safety_fault_data[FAULT_BQ40_PF_STATUS].extra_data);
                err_msg[sizeof(err_msg) - 1] = 0;
                display_draw_message(MSG_ICON_ALERT, "Pack PF", err_msg);
                break;
            case SCREEN_BQ_INITIALIZING:
                display_draw_message(MSG_ICON_INFO, "Booting", "BQ40 in Init");
                break;
            case SCREEN_BQ_SHUTDOWN:
                display_draw_message(MSG_ICON_INFO, "Shutdown", "In Progress...");
                break;
            case SCREEN_BQ_PWR_CYCLE:
                display_draw_message(MSG_ICON_INFO, "MFC Req", "Cycling Power");
                break;
            case SCREEN_BQ_XDSG:
                display_draw_message(MSG_ICON_ALERT, "Dsg Dis.", "Tap for Menu");
                break;

            // Default case where we don't know how to render this screen, better than nothing
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
    state.last_batt_state = battery_state;
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
    display_tick(core1_get_batt_state());
    state.msg_pending = false;
}

bool display_check_on(void) {
    return state.cur_target.op != OP_DISPLAY_OFF;
}
