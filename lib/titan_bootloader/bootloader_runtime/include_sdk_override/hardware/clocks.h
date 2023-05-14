/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_CLOCKS_H_
#define _HARDWARE_CLOCKS_H_

#include "pico.h"
#include "hardware/structs/clocks.h"

/**
 * @file bootloader_clocks.h
 * @brief Basic RP2040 clock implementation to configure clocks enough to provide a bootloader for the chip.
 */

#define KHZ 1000
#define MHZ 1000000

/*! \brief Initialise the clock hardware
 *  \ingroup hardware_clocks
 *
 *  Must be called before any other clock function.
 */
void clocks_init(void);

/*! \brief Get the current frequency of the specified clock
 *  \ingroup hardware_clocks
 *
 * \param clk_index Clock
 * \return Clock frequency in Hz
 */
uint32_t clock_get_hz(enum clock_index clk_index);

/*! \brief Output an optionally divided clock to the specified gpio pin.
 *  \ingroup hardware_clocks
 *
 * \param gpio The GPIO pin to output the clock to. Valid GPIOs are: 21, 23, 24, 25. These GPIOs are connected to the GPOUT0-3 clock generators.
 * \param src  The source clock. See the register field CLOCKS_CLK_GPOUT0_CTRL_AUXSRC for a full list. The list is the same for each GPOUT clock generator.
 * \param div  The amount to divide the source clock by. This is useful to not overwhelm the GPIO pin with a fast clock.
 */
void clock_gpio_init(uint gpio, uint src, uint div);

#endif
