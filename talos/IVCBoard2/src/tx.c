#include "tx.h"

#include "fft/fft.h"
#include "ivc.h"
#include "protocol.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <stdbool.h>
#include <stdint.h>

#define SYSCLK_HZ clock_get_hz(clk_sys)
#define PWM_WRAP_VALUE ((int) (((float) SYSCLK_HZ) / 5000 / 255.0f) + 1.0f)

repeating_timer_t tx_timer = { 0 };
struct QUEUE_DEFINE(uint8_t, 8) tx_msg_queue = { 0 };

void tx_init(ivc_context_t *ctx) {
    // Find out which PWM slice is connected to TX_PIN (it's slice 0)
    gpio_set_function(TX_PIN, GPIO_FUNC_PWM);
    ctx->tx.pwm_slice_num = pwm_gpio_to_slice_num(TX_PIN);
    uint pwm_chan = pwm_gpio_to_channel(TX_PIN);

    pwm_set_wrap(ctx->tx.pwm_slice_num, PWM_WRAP_VALUE - 1);
    pwm_set_chan_level(ctx->tx.pwm_slice_num, pwm_chan, PWM_WRAP_VALUE / 2);

    // Select lpf output
    gpio_init(OUTPUT_SELECT_PIN);
    gpio_set_dir(OUTPUT_SELECT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_SELECT_PIN, 1);
}
// TODO: test on hardware
// static void tx_encode_bit(ivc_context_t *ctx, uint8_t bit) {
//     float freq = (bit) ? FREQ_HIGH_HZ : FREQ_LOW_HZ;
//     // LOG_INFO("should be writing");
//     pwm_set_clkdiv(ctx->tx.pwm_slice_num, clock_get_hz(clk_sys) / (freq * PWM_WRAP_VALUE));
//     ctx->tx.flags.has_synced = false;
//     ctx->tx.packet_to_write >>= 1;
//     ctx->tx.num_bits_written++;
// }

/**
 * @brief transmits a bit over tx
 *
 * @param ctx pointer to the main context struct
 * @param bit the bit to be encoded/transmitted
 */
static void tx_encode_bit(ivc_context_t *ctx, uint8_t bit) {
    float freq = (bit) ? FREQ_HIGH_HZ : FREQ_LOW_HZ;
    // LOG_INFO("should be writing");
    pwm_set_clkdiv(ctx->tx.pwm_slice_num, clock_get_hz(clk_sys) / (freq * PWM_WRAP_VALUE));
}

/**
 * @brief transmits a sync symbol over tx
 *
 * @param ctx pointer to the main context struct
 */
static void tx_encode_sync(ivc_context_t *ctx) {
    // LOG_INFO("syncing");
    pwm_set_clkdiv(ctx->tx.pwm_slice_num, clock_get_hz(clk_sys) / (FREQ_SYNC_HZ * PWM_WRAP_VALUE));
}

/**
 * @brief disables tx
 *
 * @param ctx pointer to the main context struct
 */
static void tx_disable(ivc_context_t *ctx) {
    pwm_set_enabled(ctx->tx.pwm_slice_num, false);
    ctx->tx.flags.is_writing = false;
}

void tx_debug(ivc_context_t *ctx, uint8_t bit) {
    pwm_set_enabled(ctx->tx.pwm_slice_num, true);
    tx_encode_bit(ctx, bit);
    add_alarm_in_ms(50, tx_disable, NULL, true);
}

/**
 * @brief reset tx state after transmission complete
 *
 * @param ctx pointer to the main context struct
 */
static void tx_reset(ivc_context_t *ctx) {
    ctx->tx.num_bits_written = 0;
    ctx->tx.data_to_write = 0;
    ctx->tx.flags.has_synced = false;
    ctx->tx.flags.is_writing = false;
    ctx->tx.flags.done_writing = false;
    ctx->tx.packet_to_write = 0;
}

// TODO: test on hardware
// static bool tx_cb(repeating_timer_t *rt) {
//     ivc_context_t *ctx = (ivc_context_t *) rt->user_data;
//     tx_flags_t *flags = &ctx->tx.flags;

//     if (flags->done_writing && !flags->need_final_sync) {
//         pwm_set_enabled(ctx->tx.pwm_slice_num, false);
//         tx_reset(ctx);
//         return false;
//     }

//     if (!flags->has_synced) {
//         tx_encode_sync(ctx);
//         flags->has_synced = true;

//         if (flags->need_final_sync) {  // just wrote out last sync, we're good to reset
//             LOG_INFO("wrote out last sync");
//             flags->done_writing = true;
//             flags->need_final_sync = false;
//         }
//         return true;
//     }

//     LOG_INFO("writing out: %hu", ctx->tx.packet_to_write);

//     tx_encode_bit(ctx, ctx->tx.packet_to_write & 0x01);

//     LOG_INFO("num bits written: %hhu", ctx->tx.num_bits_written);

//     if (ctx->tx.num_bits_written == DATA_SIZE + CRC_SIZE) {
//         LOG_INFO("last bit written, need final sync");
//         flags->need_final_sync = true;
//     }

//     return true;
// }

/**
 * @brief every symbol period, transmits a bit of data and a sync, until completion
 *
 * @param rt the repeating timer struct containing the context struct
 * @return true if callback needs to execute again
 * @return false if callback shouldn't fire again
 */
static bool tx_cb(repeating_timer_t *rt) {
    ivc_context_t *ctx = (ivc_context_t *) rt->user_data;
    if (ctx->tx.flags.done_writing && !ctx->tx.flags.need_final_sync) {
        pwm_set_enabled(ctx->tx.pwm_slice_num, false);
        tx_reset(ctx);
        return false;
    }
    else {
        if (!ctx->tx.flags.has_synced) {
            tx_encode_sync(ctx);
            ctx->tx.flags.has_synced = true;
            if (ctx->tx.flags.need_final_sync) {
                LOG_INFO("wrote out last sync");
                ctx->tx.flags.done_writing = true;
                ctx->tx.flags.need_final_sync = false;
                ctx->tx.flags.has_synced = false;
            }
            return true;
        }
        LOG_INFO("writing out: %hhu", ctx->tx.data_to_write);
        tx_encode_bit(ctx, ctx->tx.data_to_write & 0x01);
        ctx->tx.flags.has_synced = false;
        ctx->tx.data_to_write >>= 1;
        ctx->tx.num_bits_written++;
        LOG_INFO("num bits written: %hhu", ctx->tx.num_bits_written);
        if (ctx->tx.num_bits_written == 8) {
            LOG_INFO("last bit written, need to sync");
            // ctx->tx.flags.done_writing = true;
            ctx->tx.flags.need_final_sync = true;
            //  return false
        }
        return true;
    }
}

void tx_enqueue_data(uint8_t data) {
    if (QUEUE_FULL(&tx_msg_queue)) {
        return;
    }
    uint8_t *entry = QUEUE_CUR_WRITE_ENTRY(&tx_msg_queue);
    *entry = data;
    LOG_INFO("enqueueing %hhu\n", *entry);
    QUEUE_MARK_WRITE_DONE(&tx_msg_queue);
}

/**
 * @brief dequeues a packet to be transmitted over tx
 *
 * @param data pointer to the callers undefined byte to be written to if data is dequeued
 * @return true if data has been dequeued
 * @return false if queue is empty
 */
static bool tx_dequeue_data(uint8_t *data) {
    if (QUEUE_EMPTY(&tx_msg_queue)) {
        // LOG_INFO("no data to pull");
        // data_ready = false;
        return false;
    }
    uint8_t *read_data = QUEUE_CUR_READ_ENTRY(&tx_msg_queue);
    *data = *read_data;
    QUEUE_MARK_READ_DONE(&tx_msg_queue);
    return true;
}

void attempt_writing(ivc_context_t *ctx) {
    if (ctx->rx.flags.receiving_packet) {
        // LOG_INFO("CANT WRITE");
        return;
    }
    // implement backoff tick count via randint % MAX_BACKOFFS
    uint8_t data;
    if (!tx_dequeue_data(&data)) {
        return;
    }
    LOG_INFO("dequeeud: %hhu ", data);
    ctx->tx.data_to_write = data;
    // TODO: test on hardware
    // ctx->tx.packet_to_write = (uint16_t) (data | calculate_crc(data) << DATA_SIZE);
    // ctx->tx.packet_to_write_copy = ctx->tx.packet_to_write;
    if (!ctx->tx.flags.is_writing) {
        pwm_set_enabled(ctx->tx.pwm_slice_num, true);
        add_repeating_timer_ms(-SYMBOL_PERIOD_MS, tx_cb, (void *) ctx, &tx_timer);
        ctx->tx.flags.is_writing = true;
    }
}
