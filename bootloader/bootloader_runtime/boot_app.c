#include <string.h>
#include <stdlib.h>
#include "pico.h"
#include "pico/bootrom.h"
#include "hardware/structs/dma.h"

extern void __boot2_start__(void);
extern void __boot_trampoline_entry__(void);
extern const void __boot_trampoline_end__;
extern const void __boot_trampoline_source__;

extern const void __flash_app;

uint32_t app_boot2_copyout[0x3F] __attribute__((aligned(256)));

void boot_app_attempt(void) {
    // Call boot2 to enable XIP
    __boot2_start__();

    const uint dma_channel = 0;

    // Configure CRC-32 on the sniffer so we can check if apps boot2 is valid
    dma_hw->sniff_data = 0xFFFFFFFF;    // Set CRC-32 initial value
    dma_hw->sniff_ctrl = (dma_channel << DMA_SNIFF_CTRL_DMACH_LSB) |
                         (DMA_SNIFF_CTRL_CALC_VALUE_CRC32 << DMA_SNIFF_CTRL_CALC_LSB) |
                         DMA_SNIFF_CTRL_EN_BITS;

    // Transfer first 252 bytes of flash app image (boot2) to USB RAM (where boot trampoline expects it)
    // This will be 63 word transfers of 4 bytes each
    uint32_t *app_boot2 = (uint32_t*)&__flash_app;

    dma_hw->ch[dma_channel].read_addr = (uintptr_t)app_boot2;
    dma_hw->ch[dma_channel].write_addr = (uintptr_t)app_boot2_copyout;
    dma_hw->ch[dma_channel].transfer_count = sizeof(app_boot2_copyout) / sizeof(*app_boot2_copyout);
    dma_hw->ch[dma_channel].ctrl_trig = DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS |
                                        (DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) |
                                        (dma_channel << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) |
                                        DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS |
                                        DMA_CH0_CTRL_TRIG_INCR_READ_BITS |
                                        (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) |
                                        DMA_CH0_CTRL_TRIG_EN_BITS;

    // Wait for transfer to complete
    while (dma_hw->ch[dma_channel].al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) {
        tight_loop_contents();
    }
    __compiler_memory_barrier();

    // Check crc32
    uint32_t computed_crc32 = dma_hw->sniff_data;
    uint32_t expected_crc32 = app_boot2[0x3F];

    // Check if boot2 has a valid CRC
    if (computed_crc32 == expected_crc32) {
        // If so boot the image

        // We need to copy out a boot trampoline to a different area of RAM (at the bottom of the stack) as we're executing in XIP cache
        // This needs to be enabled before launching the application, which we can't do here
        size_t boot_trampoline_size = ((uintptr_t)&__boot_trampoline_end__) - ((uintptr_t)&__boot_trampoline_entry__);
        memcpy(&__boot_trampoline_entry__, &__boot_trampoline_source__, boot_trampoline_size);

        // Call the trampoline. This shouldn't return
        __boot_trampoline_entry__();
        exit(1);
    }

    // If we failed, then exit XIP
    rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    assert(flash_exit_xip);
    flash_exit_xip();
}