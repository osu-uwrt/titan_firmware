#include "ivc.h"

#include "consensus.h"
#include "fft/fft.h"
#include "rx.h"
#include "storage.h"
#include "tx.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <stdint.h>

bool is_talos = false;
ivc_context_t context = { 0 };

/**
 * @brief the interupt handler for swapping the process/fill buffers and resume fft sampling
 */
static void swap_buffer_handler() {
    context.rx.recv.fft_target = context.rx.recv.buffer_select;
    context.rx.recv.buffer_select = !context.rx.recv.buffer_select;
    fft_sample(context.rx.recv.swap_buffers[context.rx.recv.buffer_select]);
}

static void swap_buffer_handler_data_ingest() {
    context.rx.recv.fft_target = context.rx.recv.buffer_select;
    context.rx.recv.buffer_select = !context.rx.recv.buffer_select;
    storage_write(context.rx.recv.swap_buffers[context.rx.recv.fft_target], NSAMP);
    fft_sample(context.rx.recv.swap_buffers[context.rx.recv.buffer_select]);
}

void ivc_init() {
    tx_init(&context);
    rx_init(&context, swap_buffer_handler);

    gpio_init(BOARD_ID_PIN);
    gpio_set_dir(BOARD_ID_PIN, GPIO_IN);

    context.is_talos = !gpio_get(BOARD_ID_PIN);
    LOG_INFO("This board %s talos", context.is_talos ? "is" : "is not");
    // set_topic_names(&ctx);
}

void ivc_tick() {
    consensus_update(&context.consensus, rx_observe(&context));
    attempt_reading(&context);
    attempt_writing(&context);
}
