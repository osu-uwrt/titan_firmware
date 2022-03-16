#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "basic_logging/logging.h"
#include "pico/time.h"

#include "drivers/async_i2c.h"
#include "drivers/adc.h"
#include "drivers/safety.h"

static int next_available_channel(struct adc_instance *inst, int next_id) {
    uint8_t polling_id_list = inst->config->monitored_channels >> next_id;
    if (polling_id_list == 0) {
        return -1;
    }

    while (!(polling_id_list & 1)) {
        polling_id_list >>= 1;
        next_id++;
    }

    return next_id;
}

// ========================================
// Polling Code
// ========================================

static void adc_channel_read_cb(const struct async_i2c_request * req);

static void adc_read_failure(const struct async_i2c_request * req, uint32_t abort_data) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;
    inst->read_in_progress = false;

    if (inst->config->failed_read_callback) {
        inst->config->failed_read_callback(req, abort_data);
    }
}

static void adc_poll_channel(struct adc_instance *inst) {
    inst->channel_to_read = next_available_channel(inst, inst->channel_to_read);
    if (inst->channel_to_read >= 0) {
        hard_assert_if(ADC, inst->channel_to_read >= 8);
        inst->tx_buffer[0] = ADC_REG_CHANNEL_DATA(inst->channel_to_read);

        async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);

    } else {
        // No channels left to process
        inst->last_reading_time = get_absolute_time();
        inst->read_in_progress = false;

        // Do final steps of initialization if first time reading
        if (!inst->initialized) {
            hard_assert_if(ADC, !inst->initializing);
            inst->active_request.failed_callback = &adc_read_failure;
            inst->initializing = false;
            inst->initialized = true;
        }
    }
}

static void adc_channel_read_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    inst->last_reading[inst->channel_to_read] = (((inst->rx_buffer[0] << 8) + inst->rx_buffer[1]) >> 4);
    inst->channel_to_read++;
    adc_poll_channel(inst);
}

static int64_t adc_poll_alarm_cb(__unused alarm_id_t id, __unused void *user_data) {
    struct adc_instance *inst = (struct adc_instance *)user_data;

    if (inst->read_in_progress) {
        LOG_ERROR("ADC Read requested with a read still in progress");
        safety_raise_fault(FAULT_ADC_ERROR);
    } else {
        inst->read_in_progress = true;
        inst->channel_to_read = 0;

        inst->active_request.completed_callback = &adc_channel_read_cb;
        inst->active_request.bytes_to_send = 1;
        inst->active_request.bytes_to_receive = 2;

        adc_poll_channel(inst);
    }

    return inst->config->poll_rate_ms * 1000;
}

// ========================================
// Initialization Code
// ========================================

static void adc_init_failure(const struct async_i2c_request * req, uint32_t abort_data){
    struct adc_instance *inst = (struct adc_instance *)req->user_data;
    hard_assert_if(ADC, inst->initialized);

    if (inst->poll_alarm_id != -1) {
        cancel_alarm(inst->poll_alarm_id);
    }

    inst->initializing = false;
    if (inst->config->failed_init_callback) {
        inst->config->failed_init_callback(req, abort_data);
    }
}

static void adc_enable_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    inst->poll_alarm_id = add_alarm_in_ms(inst->config->poll_rate_ms, &adc_poll_alarm_cb, inst, true);
    hard_assert(inst->poll_alarm_id > 0);
}

static void adc_interrupt_mask_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    // Start ADC and disable interrupts
    inst->active_request.completed_callback = &adc_enable_cb;
    inst->tx_buffer[0] = ADC_REG_CONFIGURTION;
    inst->tx_buffer[1] = 1;
    inst->active_request.bytes_to_send = 2;
    inst->active_request.bytes_to_receive = 0;

    async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
}

static void adc_channel_disable_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    // Mask All Interrupts
    inst->active_request.completed_callback = &adc_interrupt_mask_cb;
    inst->tx_buffer[0] = ADC_REG_INTERRUPT_MASK;
    inst->tx_buffer[1] = 0xFF;
    inst->active_request.bytes_to_send = 2;
    inst->active_request.bytes_to_receive = 0;

    async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
}

static void adc_conversion_rate_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    // Disable Unused Channels
    inst->active_request.completed_callback = &adc_channel_disable_cb;
    inst->tx_buffer[0] = ADC_REG_CHANNEL_DISABLE;
    inst->tx_buffer[1] = ~(inst->config->monitored_channels);
    inst->active_request.bytes_to_send = 2;
    inst->active_request.bytes_to_receive = 0;

    async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
}

static void adc_advanced_config_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    // Send continuous conversion
    inst->active_request.completed_callback = &adc_conversion_rate_cb;
    inst->tx_buffer[0] = ADC_REG_CONVERSION_RATE;
    inst->tx_buffer[1] = 1;
    inst->active_request.bytes_to_send = 2;
    inst->active_request.bytes_to_receive = 0;

    async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
}

static void adc_poll_readiness_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    if ((inst->rx_buffer[0] & 0b00000010) != 0) {
        // Loop sending request until adc is ready
        async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
    } else {

        // Send configuration
        inst->active_request.completed_callback = &adc_advanced_config_cb;
        inst->tx_buffer[0] = ADC_REG_ADVANCED_CONFIG;

        if (inst->config->enable_temperature) {
            inst->tx_buffer[1] = 0;
        } else {
            inst->tx_buffer[1] = 0b10;
        }

        if (inst->config->external_vref) {
            inst->tx_buffer[1] |= 1;
        }

        inst->active_request.bytes_to_send = 2;
        inst->active_request.bytes_to_receive = 0;

        async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
    }
}

static void adc_get_revision_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    if (inst->rx_buffer[0] != 9) {
        LOG_ERROR("ADC: Invalid Revision ID (inst: %p - ID: %x)", inst, inst->rx_buffer[0]);
        if (inst->config->failed_init_callback) {
            inst->config->failed_init_callback(req, 0);
        }
    } else {
        inst->active_request.completed_callback = &adc_poll_readiness_cb;
        inst->tx_buffer[0] = ADC_REG_BUSY_STATUS;
        inst->active_request.bytes_to_send = 1;
        inst->active_request.bytes_to_receive = 1;

        async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
    }
}

static void adc_get_manufacturer_cb(const struct async_i2c_request * req) {
    struct adc_instance *inst = (struct adc_instance *)req->user_data;

    if (inst->rx_buffer[0] != 1) {
        LOG_ERROR("ADC: Invalid Manufactuer ID (inst: %p - ID: %x)", inst, inst->rx_buffer[0]);
        if (inst->config->failed_init_callback) {
            inst->config->failed_init_callback(req, 0);
        }
    } else {
        inst->active_request.completed_callback = &adc_get_revision_cb;
        inst->tx_buffer[0] = ADC_REG_REVISION_ID;
        inst->active_request.bytes_to_send = 1;
        inst->active_request.bytes_to_receive = 1;

        async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
    }
}

void adc_start(struct adc_instance *inst, const struct adc_configuration *config) {
    invalid_params_if(ADC, inst->initialized || inst->initializing);
    invalid_params_if(ADC, config->monitored_channels == 0);
    invalid_params_if(ADC, config->poll_rate_ms == 0);
    
    inst->initialized = false;
    inst->initializing = true;
    inst->config = config;
    inst->poll_alarm_id = -1;
    inst->request_in_progress = false;

    // Initialize active_request struct to default values
    inst->active_request.i2c = config->i2c;
    inst->active_request.address = config->address;
    inst->active_request.nostop = false;
    inst->active_request.tx_buffer = inst->tx_buffer;
    inst->active_request.rx_buffer = inst->rx_buffer;
    inst->active_request.failed_callback = &adc_init_failure;
    inst->active_request.next_req_on_success = NULL;
    inst->active_request.user_data = inst;

    // Send request to get ID
    inst->active_request.completed_callback = &adc_get_manufacturer_cb;
    inst->tx_buffer[0] = ADC_REG_MANUFACTURER_ID;
    inst->active_request.bytes_to_send = 1;
    inst->active_request.bytes_to_receive = 1;

    async_i2c_enqueue(&inst->active_request, &inst->request_in_progress);
}