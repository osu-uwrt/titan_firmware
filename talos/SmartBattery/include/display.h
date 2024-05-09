#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>

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
void display_show_msg(const char *msg, const char *submsg);

bool display_check_on(void);

#endif
