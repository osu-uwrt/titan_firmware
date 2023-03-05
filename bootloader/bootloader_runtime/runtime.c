#include "hardware/clocks.h"
#include "hardware/resets.h"
#include "hardware/regs/m0plus.h"
#include "hardware/structs/padsbank0.h"

void runtime_init(void) {
    // Reset all peripherals to put system into a known state,
    // - except for QSPI pads and the XIP IO bank, as this is fatal if running from flash
    // - and the PLLs, as this is fatal if clock muxing has not been reset on this boot
    // - and USB, syscfg, as this disturbs USB-to-SWD on core 1
    reset_block(~(
            RESETS_RESET_IO_QSPI_BITS |
            RESETS_RESET_PADS_QSPI_BITS |
            RESETS_RESET_PLL_USB_BITS |
            RESETS_RESET_PLL_SYS_BITS
    ));

    // Remove reset from peripherals which are clocked only by clk_sys and
    // clk_ref. Other peripherals stay in reset until we've configured clocks.
    unreset_block_wait(RESETS_RESET_BITS & ~(
            RESETS_RESET_ADC_BITS |
            RESETS_RESET_RTC_BITS |
            RESETS_RESET_SPI0_BITS |
            RESETS_RESET_SPI1_BITS |
            RESETS_RESET_UART0_BITS |
            RESETS_RESET_UART1_BITS |
            RESETS_RESET_USBCTRL_BITS
    ));

    // pre-init runs really early since we need it even for memcpy and divide!
    // (basically anything in aeabi that uses bootrom)
    // Only reason this is in here is since we probably want to have memcpy and stuff, even in the bootrom

    // Start and end points of the constructor list,
    // defined by the linker script.
    extern void (*__preinit_array_start)(void);
    extern void (*__preinit_array_end)(void);

    // Call each function in the list.
    // We have to take the address of the symbols, as __preinit_array_start *is*
    // the first function pointer, not the address of it.
    for (void (**p)(void) = &__preinit_array_start; p < &__preinit_array_end; ++p) {
        (*p)();
    }

    // After calling preinit we have enough runtime to do the exciting maths
    // in clocks_init
    clocks_init();

    // Peripheral clocks should now all be running
    unreset_block_wait(RESETS_RESET_BITS);

    // after resetting BANK0 we should disable IE on 26-29
    hw_clear_alias(padsbank0_hw)->io[26] = hw_clear_alias(padsbank0_hw)->io[27] =
            hw_clear_alias(padsbank0_hw)->io[28] = hw_clear_alias(padsbank0_hw)->io[29] = PADS_BANK0_GPIO0_IE_BITS;

#ifndef NDEBUG
    if (__get_current_exception()) {
        // crap; started in exception handler
        __asm ("bkpt #0");
    }
#endif

    // Initialize IRQ Priorities
    static_assert(!(NUM_IRQS & 3), "");
    uint32_t prio4 = (3 & 0xff) * 0x1010101u;
    io_rw_32 * p = (io_rw_32 *)(PPB_BASE + M0PLUS_NVIC_IPR0_OFFSET);
    for (uint i = 0; i < NUM_IRQS / 4; i++) {
        *p++ = prio4;
    }

    // Don't call init_array, as this isn't needed for anything critical and just wastes space in the bootrom
    // Really only useful for C++ stuff
}

// Using _exit defined in crt0.S
extern void _exit(int status);

// exit is not useful... no desire to pull in __call_exitprocs
void exit(int status) {
    _exit(status);
}

// incorrect warning from GCC 6
void __assert_func(__unused const char *file, __unused int line, __unused const char *func, __unused const char *failedexpr) {
    // No way to print debuggnig statements in the bootloader, just abort
    _exit(1);
}

void __attribute__((noreturn)) panic_unsupported() {
    panic("not supported");
}

void __attribute__((noreturn)) __printflike(1, 0) panic(__unused const char *fmt, ...) {
    // No way to print debuggnig statements in the bootloader, just abort
    _exit(1);
}

void hard_assertion_failure(void) {
    panic("Hard assert");
}