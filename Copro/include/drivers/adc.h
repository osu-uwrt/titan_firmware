#ifndef _ADC_H
#define _ADC_H

#include <stdbool.h>
#include <stdint.h>

#include "pico/time.h"

#include "async_i2c.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_ADC, Enable/disable assertions in the ADC module, type=bool, default=0, group=Copro
#ifndef PARAM_ASSERTIONS_ENABLED_ADC
#define PARAM_ASSERTIONS_ENABLED_ADC 0
#endif

#define ADC_REG_CONFIGURTION      0x00
#define ADC_REG_INTERRUPT_STATUS  0x01
#define ADC_REG_INTERRUPT_MASK    0x03
#define ADC_REG_CONVERSION_RATE   0x07
#define ADC_REG_CHANNEL_DISABLE   0x08
#define ADC_REG_ONE_SHOT          0x09
#define ADC_REG_DEEP_SHUTDOWN     0x0A
#define ADC_REG_ADVANCED_CONFIG   0x0B
#define ADC_REG_BUSY_STATUS       0x0C
#define ADC_REG_CHANNEL_DATA(ch)  (0x20+ch)
#define ADC_REG_LIMIT(ch)         (0x2A+ch)
#define ADC_REG_MANUFACTURER_ID   0x3E
#define ADC_REG_REVISION_ID       0x3F

struct adc_configuration {
    i2c_inst_t *i2c;
    uint8_t address;
    uint8_t poll_rate_ms;
    bool enable_temperature;
    bool external_vref;
    uint8_t monitored_channels;

    async_i2c_abort_cb_t failed_init_callback;
    async_i2c_abort_cb_t failed_read_callback;
};

struct adc_instance {
    bool initialized;   // True when initialized and at least one reading has been taken
    bool initializing;  // True when in the process of initialization and has not failed

    const struct adc_configuration *config;

    // Polling Variables
    alarm_id_t poll_alarm_id;
    bool read_in_progress;
    int channel_to_read;
    uint16_t last_reading[8];
    absolute_time_t last_reading_time;

    // I2C Request Variables
    bool request_in_progress;
    struct async_i2c_request active_request;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];
};


void adc_start(struct adc_instance *inst, const struct adc_configuration *config);

inline static uint16_t adc_get_reading(struct adc_instance *inst, int channel) {
    invalid_params_if(ADC, !inst->initialized);
    invalid_params_if(ADC, !(inst->config->monitored_channels & (1<<channel)));

    return inst->last_reading[channel];
}

#endif