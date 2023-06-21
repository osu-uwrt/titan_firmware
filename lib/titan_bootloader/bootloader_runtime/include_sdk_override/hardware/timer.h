/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_TIMER_H
#define _HARDWARE_TIMER_H

#include "pico.h"
#include "hardware/structs/timer.h"

/** \file hardware/timer.h
 *  \defgroup hardware_timer hardware_timer
 *
 * Stripped down Low-level hardware timer API for bootloader operation
 */

/*! \brief Return a 32 bit timestamp value in microseconds
*  \ingroup hardware_timer
*
* Returns the low 32 bits of the hardware timer.
* \note This value wraps roughly every 1 hour 11 minutes and 35 seconds.
*
* \return the 32 bit timestamp
*/
static inline uint32_t time_us_32(void) {
    return timer_hw->timerawl;
}

/*! \brief Return the current 64 bit timestamp value in microseconds
*  \ingroup hardware_timer
*
* Returns the full 64 bits of the hardware timer. The \ref pico_time and other functions rely on the fact that this
* value monotonically increases from power up. As such it is expected that this value counts upwards and never wraps
* (we apologize for introducing a potential year 5851444 bug).
*
* \return the 64 bit timestamp
*/
uint64_t time_us_64(void);

/*! \brief Busy wait wasting cycles for the given (32 bit) number of microseconds
 *  \ingroup hardware_timer
 *
 * \param delay_us delay amount in microseconds
 */
void busy_wait_us_32(uint32_t delay_us);

/*! \brief Busy wait wasting cycles for the given (64 bit) number of microseconds
 *  \ingroup hardware_timer
 *
 * \param delay_us delay amount in microseconds
 */
void busy_wait_us(uint64_t delay_us);

/*! \brief Busy wait wasting cycles for the given number of milliseconds
 *  \ingroup hardware_timer
 *
 * \param delay_ms delay amount in milliseconds
 */
void busy_wait_ms(uint32_t delay_ms);

/*! \brief Busy wait wasting cycles until after the specified timestamp
 *  \ingroup hardware_timer
 *
 * \param t Absolute time to wait until
 */
void busy_wait_until(absolute_time_t t);

/*! \brief Check if the specified timestamp has been reached
 *  \ingroup hardware_timer
 *
 * \param t Absolute time to compare against current time
 * \return true if it is now after the specified timestamp
 */
static inline bool time_reached(absolute_time_t t) {
    uint64_t target = to_us_since_boot(t);
    uint32_t hi_target = (uint32_t)(target >> 32u);
    uint32_t hi = timer_hw->timerawh;
    return (hi >= hi_target && (timer_hw->timerawl >= (uint32_t) target || hi != hi_target));
}

#endif
