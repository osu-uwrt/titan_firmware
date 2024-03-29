#ifndef CAN_BL_CUSTOM_ID__SBH_MCU_H
#define CAN_BL_CUSTOM_ID__SBH_MCU_H

#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/addressmap.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"

#include <stdint.h>

#define SHUTOFF_ALARM_ID 0
#define SHUTOFF_ALARM_TIMEOUT_MS 5000

// Override Alarm IRQ
static_assert(TIMER_IRQ_0 + SHUTOFF_ALARM_ID == 0, "Expected timer irq to be irq 0");
void isr_irq0(void) {
    // In the event the bootloader is on for too long, this will kill power to ensure we don't accidentally drain the
    // battery This should never fire normally, but handles edge cases where it's possible the bootloader stays on (ex.
    // corrupted firmware)
    gpio_set_outover(PWR_CTRL_PIN, GPIO_OVERRIDE_LOW);
    gpio_set_oeover(PWR_CTRL_PIN, GPIO_OVERRIDE_HIGH);

    // Clear interrupt pending on alarm hardware
    timer_hw->intr = 1u << SHUTOFF_ALARM_ID;
    // Disable interrupt in nvic
    *((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ICER_OFFSET)) = (1 << (TIMER_IRQ_0 + SHUTOFF_ALARM_ID));
}

static void schedule_alarm(void) {
    // Disarm alarm in case it is running
    timer_hw->armed = (1 << SHUTOFF_ALARM_ID);

    // Clear pending IRQ in timer (in case it fired for whatever reason)
    timer_hw->intr = 1u << SHUTOFF_ALARM_ID;
    // Enable IRQ in timer hardware
    timer_hw->inte = 1u << SHUTOFF_ALARM_ID;

    // Clear pending IRQ in NVIC in case it was pending for whatever reason
    *((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ICPR_OFFSET)) = (1 << (TIMER_IRQ_0 + SHUTOFF_ALARM_ID));
    // Enable IRQ in NVIC for the alarm
    *((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ISER_OFFSET)) = (1 << (TIMER_IRQ_0 + SHUTOFF_ALARM_ID));

    // Now schedule the alarm
    static_assert(SHUTOFF_ALARM_TIMEOUT_MS < UINT32_MAX / 1000, "Timeout does not fit into 32-bit int");
    timer_hw->alarm[SHUTOFF_ALARM_ID] = time_us_32() + (SHUTOFF_ALARM_TIMEOUT_MS * 1000);
}

static bool bl_board_get_client_id(int *client_id) {
    // To make the smart battery more convenient, we will keep the power on the POWER_CTRL pin, so they can just wave
    // the switch near the battery to wake it up
    // Note we ONLY want to do this if we were woken up from a fresh start
    // If we do this on watchdog reset, we could get into a weird loop where the bootloader keeps the power always on
    if (!watchdog_caused_reboot()) {
        gpio_set_outover(PWR_CTRL_PIN, GPIO_OVERRIDE_HIGH);
        gpio_set_oeover(PWR_CTRL_PIN, GPIO_OVERRIDE_HIGH);
        gpio_disable_pulls(PWR_CTRL_PIN);

        // Schedule alarm to kill power in 5 seconds to protect the battery from accidentally draining
        // This should ideally never occur, as the bootloader should pass control over to firmware in 5 seconds
        // However, in the event the bootloader gets stuck in a loop (ex. corrupt firmware)
        // this will act as a final protection from the MCU getting stuck on
        schedule_alarm();
    }

    // Read the last sector of flash to get the programmed board id
    uint8_t data[256];
    flash_read(0x1FF000, data, sizeof(data));
    if (data[0] == 0xFF || data[0] == 0) {
        return false;
    }

    *client_id = data[0];
    return true;
}

#endif
