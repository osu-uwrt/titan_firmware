/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_GPIO_H_
#define _HARDWARE_GPIO_H_

#include "pico.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/padsbank0.h"

#ifdef __cplusplus
extern "C" {
#endif

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_GPIO, Enable/disable assertions in the GPIO module, type=bool, default=0, group=hardware_gpio
#ifndef PARAM_ASSERTIONS_ENABLED_GPIO
#define PARAM_ASSERTIONS_ENABLED_GPIO 0
#endif

/** \file gpio.h
 *  \defgroup hardware_gpio hardware_gpio
 *
 * General Purpose Input/Output (GPIO) API
 *
 * RP2040 has 36 multi-functional General Purpose Input / Output (GPIO) pins, divided into two banks. In a typical use case,
 * the pins in the QSPI bank (QSPI_SS, QSPI_SCLK and QSPI_SD0 to QSPI_SD3) are used to execute code from an external
 * flash device, leaving the User bank (GPIO0 to GPIO29) for the programmer to use. All GPIOs support digital input and
 * output, but GPIO26 to GPIO29 can also be used as inputs to the chip’s Analogue to Digital Converter (ADC). Each GPIO
 * can be controlled directly by software running on the processors, or by a number of other functional blocks.
 *
 * The function allocated to each GPIO is selected by calling the \ref gpio_set_function function. \note Not all functions
 * are available on all pins.
 *
 * Each GPIO can have one function selected at a time. Likewise, each peripheral input (e.g. UART0 RX) should only be selected on
 * one _GPIO_ at a time. If the same peripheral input is connected to multiple GPIOs, the peripheral sees the logical OR of these
 * GPIO inputs. Please refer to the datasheet for more information on GPIO function select.
 *
 * ### Function Select Table
 *
 *  GPIO   | F1       | F2        | F3       | F4     | F5  | F6   | F7   | F8            | F9
 *  -------|----------|-----------|----------|--------|-----|------|------|---------------|----
 *  0      | SPI0 RX  | UART0 TX  | I2C0 SDA | PWM0 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  1      | SPI0 CSn | UART0 RX  | I2C0 SCL | PWM0 B | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  2      | SPI0 SCK | UART0 CTS | I2C1 SDA | PWM1 A | SIO | PIO0 | PIO1 |               | USB VBUS EN
 *  3      | SPI0 TX  | UART0 RTS | I2C1 SCL | PWM1 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  4      | SPI0 RX  | UART1 TX  | I2C0 SDA | PWM2 A | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  5      | SPI0 CSn | UART1 RX  | I2C0 SCL | PWM2 B | SIO | PIO0 | PIO1 |               | USB VBUS EN
 *  6      | SPI0 SCK | UART1 CTS | I2C1 SDA | PWM3 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  7      | SPI0 TX  | UART1 RTS | I2C1 SCL | PWM3 B | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  8      | SPI1 RX  | UART1 TX  | I2C0 SDA | PWM4 A | SIO | PIO0 | PIO1 |               | USB VBUS EN
 *  9      | SPI1 CSn | UART1 RX  | I2C0 SCL | PWM4 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  10     | SPI1 SCK | UART1 CTS | I2C1 SDA | PWM5 A | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  11     | SPI1 TX  | UART1 RTS | I2C1 SCL | PWM5 B | SIO | PIO0 | PIO1 |               | USB VBUS EN
 *  12     | SPI1 RX  | UART0 TX  | I2C0 SDA | PWM6 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  13     | SPI1 CSn | UART0 RX  | I2C0 SCL | PWM6 B | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  14     | SPI1 SCK | UART0 CTS | I2C1 SDA | PWM7 A | SIO | PIO0 | PIO1 |               | USB VBUS EN
 *  15     | SPI1 TX  | UART0 RTS | I2C1 SCL | PWM7 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  16     | SPI0 RX  | UART0 TX  | I2C0 SDA | PWM0 A | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  17     | SPI0 CSn | UART0 RX  | I2C0 SCL | PWM0 B | SIO | PIO0 | PIO1 |               | USB VBUS EN
 *  18     | SPI0 SCK | UART0 CTS | I2C1 SDA | PWM1 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  19     | SPI0 TX  | UART0 RTS | I2C1 SCL | PWM1 B | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  20     | SPI0 RX  | UART1 TX  | I2C0 SDA | PWM2 A | SIO | PIO0 | PIO1 | CLOCK GPIN0   | USB VBUS EN
 *  21     | SPI0 CSn | UART1 RX  | I2C0 SCL | PWM2 B | SIO | PIO0 | PIO1 | CLOCK GPOUT0  | USB OVCUR DET
 *  22     | SPI0 SCK | UART1 CTS | I2C1 SDA | PWM3 A | SIO | PIO0 | PIO1 | CLOCK GPIN1   | USB VBUS DET
 *  23     | SPI0 TX  | UART1 RTS | I2C1 SCL | PWM3 B | SIO | PIO0 | PIO1 | CLOCK GPOUT1  | USB VBUS EN
 *  24     | SPI1 RX  | UART1 TX  | I2C0 SDA | PWM4 A | SIO | PIO0 | PIO1 | CLOCK GPOUT2  | USB OVCUR DET
 *  25     | SPI1 CSn | UART1 RX  | I2C0 SCL | PWM4 B | SIO | PIO0 | PIO1 | CLOCK GPOUT3  | USB VBUS DET
 *  26     | SPI1 SCK | UART1 CTS | I2C1 SDA | PWM5 A | SIO | PIO0 | PIO1 |               | USB VBUS EN
 *  27     | SPI1 TX  | UART1 RTS | I2C1 SCL | PWM5 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET
 *  28     | SPI1 RX  | UART0 TX  | I2C0 SDA | PWM6 A | SIO | PIO0 | PIO1 |               | USB VBUS DET
 *  29     | SPI1 CSn | UART0 RX  | I2C0 SCL | PWM6 B | SIO | PIO0 | PIO1 |               | USB VBUS EN

 */

/*! \brief  GPIO function definitions for use with function select
 *  \ingroup hardware_gpio
 * \brief GPIO function selectors
 *
 * Each GPIO can have one function selected at a time. Likewise, each peripheral input (e.g. UART0 RX) should only be
 * selected on one GPIO at a time. If the same peripheral input is connected to multiple GPIOs, the peripheral sees the logical
 * OR of these GPIO inputs.
 *
 * Please refer to the datsheet for more information on GPIO function selection.
 */
enum gpio_function {
    GPIO_FUNC_XIP = 0,
    GPIO_FUNC_SPI = 1,
    GPIO_FUNC_UART = 2,
    GPIO_FUNC_I2C = 3,
    GPIO_FUNC_PWM = 4,
    GPIO_FUNC_SIO = 5,
    GPIO_FUNC_PIO0 = 6,
    GPIO_FUNC_PIO1 = 7,
    GPIO_FUNC_GPCK = 8,
    GPIO_FUNC_USB = 9,
    GPIO_FUNC_NULL = 0x1f,
};

#define GPIO_OUT 1
#define GPIO_IN 0

/*! \brief Slew rate limiting levels for GPIO outputs
 *  \ingroup hardware_gpio
 *
 * Slew rate limiting increases the minimum rise/fall time when a GPIO output
 * is lightly loaded, which can help to reduce electromagnetic emissions.
 * \sa gpio_set_slew_rate
 */
enum gpio_slew_rate {
    GPIO_SLEW_RATE_SLOW = 0,  ///< Slew rate limiting enabled
    GPIO_SLEW_RATE_FAST = 1   ///< Slew rate limiting disabled
};

/*! \brief Drive strength levels for GPIO outputs
 *  \ingroup hardware_gpio
 *
 * Drive strength levels for GPIO outputs.
 * \sa gpio_set_drive_strength
 */
enum gpio_drive_strength {
    GPIO_DRIVE_STRENGTH_2MA = 0, ///< 2 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_4MA = 1, ///< 4 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_8MA = 2, ///< 8 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_12MA = 3 ///< 12 mA nominal drive strength
};

// ----------------------------------------------------------------------------
// Pad Controls + IO Muxing
// ----------------------------------------------------------------------------
// Declarations for gpio.c

/*! \brief Select GPIO function
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param fn Which GPIO function select to use from list \ref gpio_function
 */
static inline void gpio_set_function(uint gpio, enum gpio_function fn) {
    // Set input enable on, output disable off
    hw_write_masked(&padsbank0_hw->io[gpio],
                   PADS_BANK0_GPIO0_IE_BITS,
                   PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS
    );
    // Zero all fields apart from fsel; we want this IO to do what the peripheral tells it.
    // This doesn't affect e.g. pullup/pulldown, as these are in pad controls.
    iobank0_hw->io[gpio].ctrl = fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

/*! \brief Determine current GPIO function
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return Which GPIO function is currently selected from list \ref gpio_function
 */
static inline enum gpio_function gpio_get_function(uint gpio) {
    return (enum gpio_function) ((iobank0_hw->io[gpio].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB);
}

/*! \brief Select up and down pulls on specific GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param up If true set a pull up on the GPIO
 * \param down If true set a pull down on the GPIO
 *
 * \note On the RP2040, setting both pulls enables a "bus keep" function,
 * i.e. a weak pull to whatever is current high/low state of GPIO.
 */
static inline void gpio_set_pulls(uint gpio, bool up, bool down) {
    hw_write_masked(
            &padsbank0_hw->io[gpio],
            (bool_to_bit(up) << PADS_BANK0_GPIO0_PUE_LSB) | (bool_to_bit(down) << PADS_BANK0_GPIO0_PDE_LSB),
            PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS
    );
}

/*! \brief Set specified GPIO to be pulled up.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
static inline void gpio_pull_up(uint gpio) {
    gpio_set_pulls(gpio, true, false);
}

/*! \brief Determine if the specified GPIO is pulled up.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the GPIO is pulled up
 */
static inline bool gpio_is_pulled_up(uint gpio) {
    return (padsbank0_hw->io[gpio] & PADS_BANK0_GPIO0_PUE_BITS) != 0;
}

/*! \brief Set specified GPIO to be pulled down.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
static inline void gpio_pull_down(uint gpio) {
    gpio_set_pulls(gpio, false, true);
}

/*! \brief Determine if the specified GPIO is pulled down.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the GPIO is pulled down
 */
static inline bool gpio_is_pulled_down(uint gpio) {
    return (padsbank0_hw->io[gpio] & PADS_BANK0_GPIO0_PDE_BITS) != 0;
}

/*! \brief Disable pulls on specified GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
static inline void gpio_disable_pulls(uint gpio) {
    gpio_set_pulls(gpio, false, false);
}

/*! \brief Initialise a GPIO for (enabled I/O and set func to GPIO_FUNC_SIO)
 *  \ingroup hardware_gpio
 *
 * Clear the output enable (i.e. set to input).
 * Clear any output value.
 *
 * \param gpio GPIO number
 */
static inline void gpio_init(uint gpio) {
    sio_hw->gpio_oe_clr = 1ul << gpio;
    sio_hw->gpio_clr = 1ul << gpio;
    gpio_set_function(gpio, GPIO_FUNC_SIO);
}

// ----------------------------------------------------------------------------
// Input
// ----------------------------------------------------------------------------

/*! \brief Get state of a single specified GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return Current state of the GPIO. 0 for low, non-zero for high
 */
static inline bool gpio_get(uint gpio) {
    return !!((1ul << gpio) & sio_hw->gpio_in);
}

/*! \brief Get raw value of all GPIOs
 *  \ingroup hardware_gpio
 *
 * \return Bitmask of raw GPIO values, as bits 0-29
 */
static inline uint32_t gpio_get_all(void) {
    return sio_hw->gpio_in;
}

// ----------------------------------------------------------------------------
// Output
// ----------------------------------------------------------------------------

/*! \brief Drive high every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to set, as bits 0-29
 */
static inline void gpio_set_mask(uint32_t mask) {
    sio_hw->gpio_set = mask;
}

/*! \brief Drive low every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to clear, as bits 0-29
 */
static inline void gpio_clr_mask(uint32_t mask) {
    sio_hw->gpio_clr = mask;
}

/*! \brief Toggle every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to toggle, as bits 0-29
 */
static inline void gpio_xor_mask(uint32_t mask) {
    sio_hw->gpio_togl = mask;
}

/*! \brief Drive GPIO high/low depending on parameters
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to change, as bits 0-29
 * \param value Value to set
 *
 * For each 1 bit in \p mask, drive that pin to the value given by
 * corresponding bit in \p value, leaving other pins unchanged.
 * Since this uses the TOGL alias, it is concurrency-safe with e.g. an IRQ
 * bashing different pins from the same core.
 */
static inline void gpio_put_masked(uint32_t mask, uint32_t value) {
    sio_hw->gpio_togl = (sio_hw->gpio_out ^ value) & mask;
}

/*! \brief Drive all pins simultaneously
 *  \ingroup hardware_gpio
 *
 * \param value Bitmask of GPIO values to change, as bits 0-29
 */
static inline void gpio_put_all(uint32_t value) {
    sio_hw->gpio_out = value;
}

/*! \brief Drive a single GPIO high/low
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param value If false clear the GPIO, otherwise set it.
 */
static inline void gpio_put(uint gpio, bool value) {
    uint32_t mask = 1ul << gpio;
    if (value)
        gpio_set_mask(mask);
    else
        gpio_clr_mask(mask);
}

/*! \brief Determine whether a GPIO is currently driven high or low
 *  \ingroup hardware_gpio
 *
 * This function returns the high/low output level most recently assigned to a
 * GPIO via gpio_put() or similar. This is the value that is presented outward
 * to the IO muxing, *not* the input level back from the pad (which can be
 * read using gpio_get()).
 *
 * To avoid races, this function must not be used for read-modify-write
 * sequences when driving GPIOs -- instead functions like gpio_put() should be
 * used to atomically update GPIOs. This accessor is intended for debug use
 * only.
 *
 * \param gpio GPIO number
 * \return true if the GPIO output level is high, false if low.
 */
static inline bool gpio_get_out_level(uint gpio) {
    return !!(sio_hw->gpio_out & (1u << gpio));
}

// ----------------------------------------------------------------------------
// Direction
// ----------------------------------------------------------------------------

/*! \brief Set a number of GPIOs to output
 *  \ingroup hardware_gpio
 *
 * Switch all GPIOs in "mask" to output
 *
 * \param mask Bitmask of GPIO to set to output, as bits 0-29
 */
static inline void gpio_set_dir_out_masked(uint32_t mask) {
    sio_hw->gpio_oe_set = mask;
}

/*! \brief Set a number of GPIOs to input
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input, as bits 0-29
 */
static inline void gpio_set_dir_in_masked(uint32_t mask) {
    sio_hw->gpio_oe_clr = mask;
}

/*! \brief Set multiple GPIO directions
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input, as bits 0-29
 * \param value Values to set
 *
 * For each 1 bit in "mask", switch that pin to the direction given by
 * corresponding bit in "value", leaving other pins unchanged.
 * E.g. gpio_set_dir_masked(0x3, 0x2); -> set pin 0 to input, pin 1 to output,
 * simultaneously.
 */
static inline void gpio_set_dir_masked(uint32_t mask, uint32_t value) {
    sio_hw->gpio_oe_togl = (sio_hw->gpio_oe ^ value) & mask;
}

/*! \brief Set direction of all pins simultaneously.
 *  \ingroup hardware_gpio
 *
 * \param values individual settings for each gpio; for GPIO N, bit N is 1 for out, 0 for in
 */
static inline void gpio_set_dir_all_bits(uint32_t values) {
    sio_hw->gpio_oe = values;
}

/*! \brief Set a single GPIO direction
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param out true for out, false for in
 */
static inline void gpio_set_dir(uint gpio, bool out) {
    uint32_t mask = 1ul << gpio;
    if (out)
        gpio_set_dir_out_masked(mask);
    else
        gpio_set_dir_in_masked(mask);
}

/*! \brief Check if a specific GPIO direction is OUT
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the direction for the pin is OUT
 */
static inline bool gpio_is_dir_out(uint gpio) {
    return !!(sio_hw->gpio_oe & (1u << (gpio)));
}

/*! \brief Get a specific GPIO direction
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return 1 for out, 0 for in
 */
static inline uint gpio_get_dir(uint gpio) {
    return gpio_is_dir_out(gpio); // note GPIO_OUT is 1/true and GPIO_IN is 0/false anyway
}

#endif // _GPIO_H_
