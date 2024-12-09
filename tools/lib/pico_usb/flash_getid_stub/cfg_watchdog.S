#include "hardware/regs/addressmap.h"
#include "hardware/regs/watchdog.h"
#include "titan/safety_magic_values.h"

// Configures the watchdog registers to report that it has been reset via the USB bootrom
// Because the bootrom doesn't set any magic values after it's been reset, we'll use this so the firmware
// doesn't report the generic Unknown Watchdog Reset when it starts up firmware

.cpu cortex-m0plus
.text
.thumb

.type entry,%function
.global entry
.thumb_func
entry:
    ldr r0, =CLEAN_BOOT
    ldr r1, =WATCHDOG_BASE
    str r0, [r1, #WATCHDOG_SCRATCH0_OFFSET]
    ldr r0, =CLEAN_RESET_TYPE_BOOTROM
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]
    bx lr
