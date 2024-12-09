#include "pico/asm_helper.S"
#include "hardware/regs/io_bank0.h"

pico_default_asm_setup

# This must be as early as possible
# As preinit is the first thing called after peripherals are reset
# However, when we reset the peripherals after starting up from bootloader, we reset the iobank, clearing the power pin
# back to an input.
# We must immediately restore power, as we are running on the residual power in the capacitors on the regulators
# Hence, assembly at an extremely early part of initialization to set the pins back to normal

__pre_init pre_init_pwr_pin, 00000

#if PWR_CTRL_PIN != 9
#error Unexpected power control pin, gpio power pin hardcoded to gpio 9
#endif

.section .text
regular_func pre_init_pwr_pin
    ldr r1, =(IO_BANK0_BASE + IO_BANK0_GPIO9_CTRL_OFFSET)
    ldr r0, =((IO_BANK0_GPIO9_CTRL_RESET & ~(IO_BANK0_GPIO9_CTRL_OEOVER_BITS | IO_BANK0_GPIO9_CTRL_OUTOVER_BITS)) | \
              (IO_BANK0_GPIO9_CTRL_OUTOVER_VALUE_HIGH << IO_BANK0_GPIO9_CTRL_OUTOVER_LSB) | \
              (IO_BANK0_GPIO9_CTRL_OEOVER_VALUE_ENABLE << IO_BANK0_GPIO9_CTRL_OEOVER_LSB))
    str r0, [r1]
    bx lr
