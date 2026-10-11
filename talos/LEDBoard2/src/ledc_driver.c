#include "ledc_driver.h"

#include "ledc_canmore.h"
#include "ledc_commands.h"

void set_brightness() {}

void ledc_init() {
    register_canmore_commands();
    set_controllers_active_mode();
    enable_dimming();
    set_peak_current();
}
