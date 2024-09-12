#ifndef SCREEN_H
#define SCREEN_H

// #include "core1.h"

#include <stdbool.h>

/**
 * @brief Initialize LCD screen
 *
 */
void screen_init(void);

/**
 * @brief Tick screen update which need to occur within screen
 *
 * @param battery_charging_state
 */
// TODO: add "batt chrgr_state_t" in main.c
void screen_tick(batt_chrgr_state_t battery_charger_state);

/**
 * @brief Show status message to screen
 */
void screen_show_msg(const char *msg, const char *submsg);

bool screen_check_on(void);

#endif
