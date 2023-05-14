/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_TIME_H
#define _PICO_TIME_H

#include "pico.h"
#include "hardware/timer.h"

/** \file time.h
 *  \defgroup pico_time pico_time
 *
 * API for accurate timestamps, sleeping, and time based callbacks
 *
 * \note The functions defined here provide a much more powerful and user friendly wrapping around the
 * low level hardware timer functionality. For these functions (and any other SDK functionality
 * e.g. timeouts, that relies on them) to work correctly, the hardware timer should not be modified. i.e. it is expected
 * to be monotonically increasing once per microsecond. Fortunately there is no need to modify the hardware
 * timer as any functionality you can think of that isn't already covered here can easily be modelled
 * by adding or subtracting a constant value from the unmodified hardware timer.
 *
 * \sa \ref hardware_timer
 */

/*!
 * \defgroup timestamp timestamp
 *  \ingroup pico_time
 * \brief Timestamp functions relating to points in time (including the current time)
 *
 * These are functions for dealing with timestamps (i.e. instants in time) represented by the type absolute_time_t. This opaque
 * type is provided to help prevent accidental mixing of timestamps and relative time values.
 */

/*! \brief Return a representation of the current time.
 * \ingroup timestamp
 *
 * Returns an opaque high fidelity representation of the current time sampled during the call.
 *
 * \return the absolute time (now) of the hardware timer
 *
 * \sa absolute_time_t
 * \sa sleep_until()
 * \sa time_us_64()
 */
static inline absolute_time_t get_absolute_time(void) {
    absolute_time_t t;
    update_us_since_boot(&t, time_us_64());
    return t;
}

static inline uint32_t us_to_ms(uint64_t us) {
    if (us >> 32u) {
        return (uint32_t)(us / 1000u);
    } else {
        return ((uint32_t)us) / 1000u;
    }
}

/*! fn to_ms_since_boot
 * \ingroup timestamp
 * \brief Convert a timestamp into a number of milliseconds since boot.
 * \param t an absolute_time_t value to convert
 * \return the number of microseconds since boot represented by t
 * \sa to_us_since_boot()
 */
static inline uint32_t to_ms_since_boot(absolute_time_t t) {
    uint64_t us = to_us_since_boot(t);
    return us_to_ms(us);
}

/*! \brief Return a timestamp value obtained by adding a number of microseconds to another timestamp
 * \ingroup timestamp
 *
 * \param t the base timestamp
 * \param us the number of microseconds to add
 * \return the timestamp representing the resulting time
 */
static inline absolute_time_t delayed_by_us(const absolute_time_t t, uint64_t us) {
    absolute_time_t t2;
    uint64_t base = to_us_since_boot(t);
    uint64_t delayed = base + us;
    if (delayed < base) {
        delayed = (uint64_t)-1;
    }
    update_us_since_boot(&t2, delayed);
    return t2;
}

/*! \brief Return a timestamp value obtained by adding a number of milliseconds to another timestamp
 * \ingroup timestamp
 *
 * \param t the base timestamp
 * \param ms the number of milliseconds to add
 * \return the timestamp representing the resulting time
 */
static inline absolute_time_t delayed_by_ms(const absolute_time_t t, uint32_t ms) {
    absolute_time_t t2;
    uint64_t base = to_us_since_boot(t);
    uint64_t delayed = base + ms * 1000ull;
    if (delayed < base) {
        delayed = (uint64_t)-1;
    }
    update_us_since_boot(&t2, delayed);
    return t2;
}

/*! \brief Convenience method to get the timestamp a number of microseconds from the current time
 * \ingroup timestamp
 *
 * \param us the number of microseconds to add to the current timestamp
 * \return the future timestamp
 */
static inline absolute_time_t make_timeout_time_us(uint64_t us) {
    return delayed_by_us(get_absolute_time(), us);
}

/*! \brief Convenience method to get the timestamp a number of milliseconds from the current time
 * \ingroup timestamp
 *
 * \param ms the number of milliseconds to add to the current timestamp
 * \return the future timestamp
 */
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) {
    return delayed_by_ms(get_absolute_time(), ms);
}

/*! \brief Return the difference in microseconds between two timestamps
 * \ingroup timestamp
 *
 * \note be careful when diffing against large timestamps (e.g. \ref at_the_end_of_time)
 * as the signed integer may overflow.
 *
 * \param from the first timestamp
 * \param to the second timestamp
 * \return the number of microseconds between the two timestamps (positive if `to` is after `from` except
 * in case of overflow)
 */
static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return (int64_t)(to_us_since_boot(to) - to_us_since_boot(from));
}

/*! \brief The timestamp representing the end of time; this is actually not the maximum possible
 * timestamp, but is set to 0x7fffffff_ffffffff microseconds to avoid sign overflows with time
 * arithmetic. This is still over 7 million years, so should be sufficient.
 * \ingroup timestamp
 */
extern const absolute_time_t at_the_end_of_time;

/*! \brief The timestamp representing a null timestamp
 * \ingroup timestamp
 */
extern const absolute_time_t nil_time;

/*! \brief Determine if the given timestamp is nil
 * \ingroup timestamp
 *  \param t the timestamp
 *  \return true if the timestamp is nil
 *  \sa nil_time
 */
static inline bool is_nil_time(absolute_time_t t) {
    return !to_us_since_boot(t);
}

#endif
