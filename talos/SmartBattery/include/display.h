#ifndef DISPLAY_H
#define DISPLAY_H
#include "pico/time.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize LCD screen
 *
 */
void display_init(void);

/**
 * @brief Tick screen update which need to occur within display
 */
void display_tick();

/**
 * @brief Show status message to screen
 */
void display_show_msg(const char *msg);

bool display_check_on(void);

#endif
