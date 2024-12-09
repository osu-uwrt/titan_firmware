#ifndef DISPLAY_H
#define DISPLAY_H

#include "core1.h"

#include <stdbool.h>

/**
 * @brief Initialize LCD screen
 *
 */
void display_init(void);

/**
 * @brief Tick screen update which need to occur within display
 *
 * @param battery_state
 */
void display_tick(batt_state_t battery_state);

/**
 * @brief Show status message to screen
 */
void display_show_msg(const char *msg, const char *submsg);

bool display_check_on(void);

#endif
