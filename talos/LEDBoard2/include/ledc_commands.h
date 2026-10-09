#ifndef LEDC_COMMANDS_H
#define LEDC_COMMANDS_H

#define BUCK_PEAK_CURRENT 45

#define THERMISTOR_SERIES_RESISTANCE 10000.0
#define THERMISTOR_V_REF 3.3
#define THERMISTOR_R_25 22000.0
#define THERMISTOR_B_25_85 3730.0
#define THERMISTOR_NOMINAL_TEMP 298.15

void pet_watchdog();
void enable_dimming();
void set_controllers_active_mode();
void set_peak_current();
float read_temp();

#endif  // LEDC_COMMANDS_H
