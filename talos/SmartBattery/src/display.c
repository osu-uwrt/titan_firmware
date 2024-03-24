#include "display.h"

#include "core1.h"
#include "uwrt_logo.h"

#include "driver/ssd1306.h"
#include "pico/time.h"

#include <stdbool.h>
#include <stdio.h>

#define DISPLAY_UPDATE_INTERVAL_MS 500
#define DISPLAY_HOLD_MS 4000

absolute_time_t next_display_update = { 0 };
absolute_time_t display_poweroff_time;
absolute_time_t lcd_poweron_delay = { 0 };
absolute_time_t soc_wake_time = { 0 };

static uint8_t menu_option = 0;

static uint16_t sweep = 90;

static uint8_t hold_count = 0;

static bool next_menu_option = false;

// Indicate the presence of input
static bool battery_input_hold = false;

bool display_on = false;

static void display_show_menu(uint8_t op_hl) {
    display_on = true;
    if (battery_input_hold)
        display_poweroff_time = make_timeout_time_ms(5000);
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    char buf[16];

    ssd1306_FillRectangle(0, op_hl * 8, SSD1306_WIDTH - 1, (op_hl + 1) * 8 - 1, White);

    // Progress circle indicate the presence of input
    if (battery_input_hold) {
        if (op_hl == 0) {
            ssd1306_DrawArc(122, 4, 5, 0, sweep, Black);
        }
        else {
            ssd1306_DrawArc(122, 4, 5, 0, sweep, White);
        }
    }

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "1, Relative SOC");
    SSD1306_COLOR color = (op_hl == 0) ? Black : White;
    ssd1306_WriteString(buf, Font_6x8, color);

    ssd1306_SetCursor(0, 8);
    snprintf(buf, sizeof(buf), "2. Voltage");
    color = (op_hl == 1) ? Black : White;
    ssd1306_WriteString(buf, Font_6x8, color);

    ssd1306_SetCursor(0, 16);
    snprintf(buf, sizeof(buf), "3. Current");
    color = (op_hl == 2) ? Black : White;
    ssd1306_WriteString(buf, Font_6x8, color);

    ssd1306_SetCursor(0, 24);
    snprintf(buf, sizeof(buf), "4. Remain Time");
    color = (op_hl == 3) ? Black : White;
    ssd1306_WriteString(buf, Font_6x8, color);

    ssd1306_UpdateScreen();
}

static void display_show_option(unsigned int serial, uint8_t op_hl, bool dsg_mode) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(7000);

    char buf[16];
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "ID: 2023-%d", serial);
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(120, 2);
    snprintf(buf, sizeof(buf), "%d", serial);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_DrawRectangle(118, 0, 127, 11, White);

    ssd1306_FillRectangle(0, 11, 79, 32, White);
    ssd1306_SetCursor(2, 13);
    if (op_hl == 0)
        snprintf(buf, sizeof(buf), "SOC %d%%", core1_soc());
    else if (op_hl == 1)
        snprintf(buf, sizeof(buf), "%.2f V", (float) core1_voltage() / 1000.0);
    else if (op_hl == 2)
        snprintf(buf, sizeof(buf), "%.2f A", (float) core1_current() / 1000.0);
    else if (op_hl == 3)
        snprintf(buf, sizeof(buf), "%d'", core1_time_to_empty());
    ssd1306_WriteString(buf, Font_11x18, Black);

    ssd1306_SetCursor(86, 22);
    if (dsg_mode)
        snprintf(buf, sizeof(buf), "DSG MD");
    else
        snprintf(buf, sizeof(buf), "CHG MD");
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void display_init(void) {
    ssd1306_Init();

    display_on = true;
    display_poweroff_time = make_timeout_time_ms(5000);
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Ohio State", Font_7x10, White);
    ssd1306_SetCursor(0, 11);
    ssd1306_WriteString("Underwater", Font_7x10, White);
    ssd1306_SetCursor(0, 22);
    ssd1306_WriteString("Robotics", Font_7x10, White);
    ssd1306_DrawBitmap(79, 0, uwrt_logo_bin, uwrt_logo_bin_width, uwrt_logo_bin_height, White);
    ssd1306_UpdateScreen();

    // Force display to refresh for the first 5s of poweron
    lcd_poweron_delay = make_timeout_time_ms(5000);
}

void display_tick(uint16_t serial) {
    if (time_reached(next_display_update)) {
        if (gpio_get(SWITCH_SIGNAL_PIN) || !time_reached(lcd_poweron_delay)) {
            // Display SOC first upon turning on
            if (!display_on) {
                // Display hold for at least 4 sec,
                // but remains on for total of 7 sec (refer to display_poweroff_time in display_show_option func)
                // allowing 3 sec for users to interact with the display
                soc_wake_time = make_timeout_time_ms(DISPLAY_HOLD_MS);
                display_show_option(serial, 0, core1_dsg_mode());
                next_display_update = make_timeout_time_ms(DISPLAY_UPDATE_INTERVAL_MS);
            }
            // Typical operation of display
            else if (time_reached(soc_wake_time)) {
                if (next_menu_option) {
                    // Reset hold_count and progress circle
                    next_menu_option = false;
                    sweep = 90;
                    hold_count = 0;
                }
                else {
                    sweep = (sweep == 360) ? 360 : (sweep + 90);
                    hold_count = (hold_count == 3) ? 3 : (hold_count + 1);
                }
                if (!battery_input_hold) {
                    battery_input_hold = true;
                }
                if (hold_count == 3 && battery_input_hold) {
                    // Display hold for at least 4 sec,
                    // but remains on for total of 7 sec (refer to display_poweroff_time in display_show_option func)
                    // allowing 3 sec for users to interact with the display
                    display_show_option(serial, menu_option, core1_dsg_mode());
                    next_display_update = make_timeout_time_ms(DISPLAY_HOLD_MS);
                    sweep = 0;
                    hold_count = 0;
                }
                else {
                    display_show_menu(menu_option);
                    next_display_update = make_timeout_time_ms(DISPLAY_UPDATE_INTERVAL_MS);
                }
            }
        }
        else if (!time_reached(display_poweroff_time)) {
            // Move on to the next option once iff there was a presence of input but not currently
            if (time_reached(soc_wake_time)) {
                next_menu_option = true;
                if (battery_input_hold)
                    menu_option = (menu_option + 1) % 4;
                display_show_menu(menu_option);
            }
            battery_input_hold = false;
        }
        else {
            menu_option = 0;
        }
    }
}

void display_show_ros_connect(void) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(2000);
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 7);
    ssd1306_WriteString("ROS Connect", Font_11x18, White);
    ssd1306_UpdateScreen();
}

void display_show_ros_disconnect(void) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(2000);
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 7);
    ssd1306_WriteString("ROS Lost", Font_11x18, White);
    ssd1306_UpdateScreen();
}

void display_check_poweroff(void) {
    if (display_on && time_reached(display_poweroff_time)) {
        display_on = false;
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        ssd1306_SetDisplayOn(0);
    }
}
