#include "display.h"

#include "core1.h"
#include "uwrt_logo.h"

#include "driver/ssd1306.h"
#include "pico/time.h"

#include <stdbool.h>
#include <stdio.h>

bool display_on = false;
absolute_time_t display_poweroff_time;
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
}

void display_show_stats(unsigned int serial, unsigned int soc, float voltage) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(10000);

    char buf[16];
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "ID: 2023-%d", serial);  // TODO: Replace with real number
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(120, 2);
    snprintf(buf, sizeof(buf), "%d", serial);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_DrawRectangle(118, 0, 127, 11, White);

    ssd1306_FillRectangle(0, 11, 79, 32, White);
    ssd1306_SetCursor(2, 13);
    snprintf(buf, sizeof(buf), "SOC %d%%", soc);
    ssd1306_WriteString(buf, Font_11x18, Black);

    ssd1306_SetCursor(86, 22);
    snprintf(buf, sizeof(buf), "%.2fV", voltage);
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void display_show_menu(uint8_t op_hl) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(10000);
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    char buf[16];

    ssd1306_FillRectangle(0, op_hl * 8, SSD1306_WIDTH - 1, (op_hl + 1) * 8 - 1, White);

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

void display_show_option(unsigned int serial, uint8_t op_hl, bool dsg_mode) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(10000);

    char buf[16];
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "ID: 2023-%d", serial);  // TODO: Replace with real number
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
        snprintf(buf, sizeof(buf), "%d'", core1_remaining_time());
    ssd1306_WriteString(buf, Font_11x18, Black);

    ssd1306_SetCursor(86, 22);
    if (dsg_mode)
        snprintf(buf, sizeof(buf), "DSG MD");
    else
        snprintf(buf, sizeof(buf), "CHG MD");
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void display_show_soc(unsigned int serial, uint8_t soc, bool dsg_mode) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(10000);

    char buf[16];
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "ID: 2023-%d", serial);  // TODO: Replace with real number
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(120, 2);
    snprintf(buf, sizeof(buf), "%d", serial);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_DrawRectangle(118, 0, 127, 11, White);

    ssd1306_FillRectangle(0, 11, 79, 32, White);
    ssd1306_SetCursor(2, 13);
    snprintf(buf, sizeof(buf), "SOC %d%%", soc);
    ssd1306_WriteString(buf, Font_11x18, Black);

    ssd1306_SetCursor(86, 22);
    if (dsg_mode)
        snprintf(buf, sizeof(buf), "DSG MD");
    else
        snprintf(buf, sizeof(buf), "CHG MD");
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void display_show_voltage(unsigned int serial, float voltage, bool dsg_mode) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(10000);

    char buf[16];
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "ID: 2023-%d", serial);  // TODO: Replace with real number
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(120, 2);
    snprintf(buf, sizeof(buf), "%d", serial);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_DrawRectangle(118, 0, 127, 11, White);

    ssd1306_FillRectangle(0, 11, 79, 32, White);
    ssd1306_SetCursor(2, 13);
    snprintf(buf, sizeof(buf), "%.2f V", voltage);
    ssd1306_WriteString(buf, Font_11x18, Black);

    ssd1306_SetCursor(86, 22);
    if (dsg_mode)
        snprintf(buf, sizeof(buf), "DSG MD");
    else
        snprintf(buf, sizeof(buf), "CHG MD");
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void display_show_current(unsigned int serial, float current, bool dsg_mode) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(10000);

    char buf[16];
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "ID: 2023-%d", serial);  // TODO: Replace with real number
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(120, 2);
    snprintf(buf, sizeof(buf), "%d", serial);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_DrawRectangle(118, 0, 127, 11, White);

    ssd1306_FillRectangle(0, 11, 79, 32, White);
    ssd1306_SetCursor(2, 13);
    snprintf(buf, sizeof(buf), "%.2f A", current);
    ssd1306_WriteString(buf, Font_11x18, Black);

    ssd1306_SetCursor(86, 22);
    if (dsg_mode)
        snprintf(buf, sizeof(buf), "DSG MD");
    else
        snprintf(buf, sizeof(buf), "CHG MD");
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void display_show_remain_time(unsigned int serial, uint16_t remain_time, bool dsg_mode) {
    display_on = true;
    display_poweroff_time = make_timeout_time_ms(10000);

    char buf[16];
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(buf, sizeof(buf), "ID: 2023-%d", serial);  // TODO: Replace with real number
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(120, 2);
    snprintf(buf, sizeof(buf), "%d", serial);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_DrawRectangle(118, 0, 127, 11, White);

    ssd1306_FillRectangle(0, 11, 79, 32, White);
    ssd1306_SetCursor(2, 13);
    snprintf(buf, sizeof(buf), "%d'", remain_time);
    ssd1306_WriteString(buf, Font_11x18, Black);

    ssd1306_SetCursor(86, 22);
    if (dsg_mode)
        snprintf(buf, sizeof(buf), "DSG MD");
    else
        snprintf(buf, sizeof(buf), "CHG MD");
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
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
