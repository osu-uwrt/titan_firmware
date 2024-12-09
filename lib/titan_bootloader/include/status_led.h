#ifndef STATUS_LED_H_
#define STATUS_LED_H_

enum status_led_state { LED_OFF, LED_BOOT_DELAY, LED_BL_NORMAL, LED_BL_FLASH, LED_HW_FAIL };

void status_led_init(void);
void status_led_set(enum status_led_state state);

#endif
