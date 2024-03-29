/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "hardware/structs/watchdog.h"
#include "hardware/regs/clocks.h"
#include "hardware/platform_defs.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/xosc.h"

static uint32_t configured_freq[CLK_COUNT];

// Clock muxing consists of two components:
// - A glitchless mux, which can be switched freely, but whose inputs must be
//   free-running
// - An auxiliary (glitchy) mux, whose output glitches when switched, but has
//   no constraints on its inputs
// Not all clocks have both types of mux.
static inline bool has_glitchless_mux(enum clock_index clk_index) {
    return clk_index == clk_sys || clk_index == clk_ref;
}

/// \tag::clock_configure[]
bool clock_configure(enum clock_index clk_index, uint32_t src, uint32_t auxsrc, uint32_t src_freq, uint32_t freq) {
    uint32_t div;

    assert(src_freq >= freq);

    if (freq > src_freq)
        return false;

    // Div register is 24.8 int.frac divider so multiply by 2^8 (left shift by 8)
    div = (uint32_t) (((uint64_t) src_freq << 8) / freq);

    clock_hw_t *clock = &clocks_hw->clk[clk_index];

    // If increasing divisor, set divisor before source. Otherwise set source
    // before divisor. This avoids a momentary overspeed when e.g. switching
    // to a faster source and increasing divisor to compensate.
    if (div > clock->div)
        clock->div = div;

    // If switching a glitchless slice (ref or sys) to an aux source, switch
    // away from aux *first* to avoid passing glitches when changing aux mux.
    // Assume (!!!) glitchless source 0 is no faster than the aux source.
    if (has_glitchless_mux(clk_index) && src == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX) {
        hw_clear_bits(&clock->ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
        while (!(clock->selected & 1u))
            tight_loop_contents();
    }
    // If no glitchless mux, cleanly stop the clock to avoid glitches
    // propagating when changing aux mux. Note it would be a really bad idea
    // to do this on one of the glitchless clocks (clk_sys, clk_ref).
    else {
        // Disable clock. On clk_ref and clk_sys this does nothing,
        // all other clocks have the ENABLE bit in the same position.
        hw_clear_bits(&clock->ctrl, CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        if (configured_freq[clk_index] > 0) {
            // Delay for 3 cycles of the target clock, for ENABLE propagation.
            // Note XOSC_COUNT is not helpful here because XOSC is not
            // necessarily running, nor is timer... so, 3 cycles per loop:
            uint delay_cyc = configured_freq[clk_sys] / configured_freq[clk_index] + 1;
            asm volatile (
                ".syntax unified \n\t"
                "1: \n\t"
                "subs %0, #1 \n\t"
                "bne 1b"
                : "+r" (delay_cyc)
            );
        }
    }

    // Set aux mux first, and then glitchless mux if this clock has one
    hw_write_masked(&clock->ctrl,
        (auxsrc << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB),
        CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS
    );

    if (has_glitchless_mux(clk_index)) {
        hw_write_masked(&clock->ctrl,
            src << CLOCKS_CLK_REF_CTRL_SRC_LSB,
            CLOCKS_CLK_REF_CTRL_SRC_BITS
        );
        while (!(clock->selected & (1u << src)))
            tight_loop_contents();
    }

    // Enable clock. On clk_ref and clk_sys this does nothing,
    // all other clocks have the ENABLE bit in the same position.
    hw_set_bits(&clock->ctrl, CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

    // Now that the source is configured, we can trust that the user-supplied
    // divisor is a safe value.
    clock->div = div;

    // Store the configured frequency
    configured_freq[clk_index] = (uint32_t)(((uint64_t) src_freq << 8) / div);

    return true;
}
/// \end::clock_configure[]

void clocks_init(void) {
    // Start tick in watchdog
    watchdog_start_tick(XOSC_MHZ);

    // Everything is 48MHz on FPGA apart from RTC. Otherwise set to 0 and will be set in clock configure
    if (running_on_fpga()) {
        for (uint i = 0; i < CLK_COUNT; i++) {
            configured_freq[i] = 48 * MHZ;
        }
        configured_freq[clk_rtc] = 46875;
        return;
    }

    // Disable resus that may be enabled from previous software
    clocks_hw->resus.ctrl = 0;

    // Enable the xosc
    xosc_init();

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1)
        tight_loop_contents();
    hw_clear_bits(&clocks_hw->clk[clk_ref].ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_ref].selected != 0x1)
        tight_loop_contents();

    /// \tag::pll_settings[]
    // Configure PLLs
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
    /// \end::pll_settings[]

    /// \tag::pll_init[]
    pll_init(pll_sys, 1, 1500 * MHZ, 6, 2);
    pll_init(pll_usb, 1, 480 * MHZ, 5, 2);
    /// \end::pll_init[]

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    0, // No aux mux
                    12 * MHZ,
                    12 * MHZ);

    /// \tag::configure_clk_sys[]
    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    125 * MHZ,
                    125 * MHZ);
    /// \end::configure_clk_sys[]

    // CLK USB = PLL USB (48MHz) / 1 = 48MHz
    clock_configure(clk_usb,
                    0, // No GLMUX
                    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    clock_configure(clk_adc,
                    0, // No GLMUX
                    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    clock_configure(clk_rtc,
                    0, // No GLMUX
                    CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    46875);

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    125 * MHZ,
                    125 * MHZ);
}

/// \tag::clock_get_hz[]
uint32_t clock_get_hz(enum clock_index clk_index) {
    return configured_freq[clk_index];
}
/// \end::clock_get_hz[]

void clock_set_reported_hz(enum clock_index clk_index, uint hz) {
    configured_freq[clk_index] = hz;
}

/// \tag::frequency_count_khz[]
uint32_t frequency_count_khz(uint src) {
    fc_hw_t *fc = &clocks_hw->fc0;

    // If frequency counter is running need to wait for it. It runs even if the source is NULL
    while(fc->status & CLOCKS_FC0_STATUS_RUNNING_BITS) {
        tight_loop_contents();
    }

    // Set reference freq
    fc->ref_khz = clock_get_hz(clk_ref) / 1000;

    fc->interval = 10;

    // No min or max
    fc->min_khz = 0;
    fc->max_khz = 0xffffffff;

    // Set SRC which automatically starts the measurement
    fc->src = src;

    while(!(fc->status & CLOCKS_FC0_STATUS_DONE_BITS)) {
        tight_loop_contents();
    }

    // Return the result
    return fc->result >> CLOCKS_FC0_RESULT_KHZ_LSB;
}
/// \end::frequency_count_khz[]

void clock_gpio_init(uint gpio, uint src, uint div) {
    // Bit messy but it's as much code to loop through a lookup
    // table. The sources for each gpout generators are the same
    // so just call with the sources from GP0
    uint gpclk = 0;
    if      (gpio == 21) gpclk = clk_gpout0;
    else if (gpio == 23) gpclk = clk_gpout1;
    else if (gpio == 24) gpclk = clk_gpout2;
    else if (gpio == 25) gpclk = clk_gpout3;
    else panic("Invalid clock");

    // Set up the gpclk generator
    clocks_hw->clk[gpclk].ctrl = (src << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB) |
                                 CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS;
    clocks_hw->clk[gpclk].div = div << CLOCKS_CLK_GPOUT0_DIV_INT_LSB;

    // Set gpio pin to gpclock function
    gpio_set_function(gpio, GPIO_FUNC_GPCK);
}
