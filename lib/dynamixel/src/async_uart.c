#include "dynamixel/async_uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/sync.h"
#include "pico/time.h"
#include "uart_multidrop.pio.h"

#define ASYNC_UART_DMA_IRQ_NUM 0
#define ASYNC_UART_PIO_IRQ_NUM 0

static_assert(ASYNC_UART_DMA_IRQ_NUM == 0 || ASYNC_UART_DMA_IRQ_NUM == 1, "ASYNC_UART_DMA_IRQ_NUM must be valid DMA IRQ number");
static_assert(ASYNC_UART_PIO_IRQ_NUM == 0 || ASYNC_UART_PIO_IRQ_NUM == 1, "ASYNC_UART_PIO_IRQ_NUM must be valid PIO IRQ number");
#define ASYNC_UART_DMA_IRQ __CONCAT(DMA_IRQ_, ASYNC_UART_DMA_IRQ_NUM)
#define ASYNC_UQRT_PIO_IRQ(pio) (pio == pio0 ? __CONCAT(PIO0_IRQ_, ASYNC_UART_PIO_IRQ_NUM) : __CONCAT(PIO1_IRQ_, ASYNC_UART_PIO_IRQ_NUM))

#define uart_multidrop_irq_tx_compl_num(sm) ((sm + uart_multidrop_irq_tx_compl_rel) % 4)
#define uart_multidrop_irq_tx_compl(sm) (pis_interrupt0 + uart_multidrop_irq_tx_compl_num(sm))
#define uart_multidrop_irq_frame_error_num(sm) ((sm + uart_multidrop_irq_frame_error_rel) % 4)
#define uart_multidrop_irq_frame_error(sm) (pis_interrupt0 + uart_multidrop_irq_frame_error_num(sm))

struct async_uart_inst {
    PIO pio;
    uint sm;
    uint offset;
    uint timeout_ms;
    async_uart_on_write tx_cb;
    async_uart_on_read rx_cb;
    uint dma_tx_chan;
    uint dma_rx_chan;
    volatile bool rx_active;
    volatile bool tx_active;
    uint8_t *rx_data_ptr;
    size_t rx_data_len;
    alarm_id_t timeout_alarm;
} local_inst;
struct async_uart_inst * const inst = &local_inst;

bool __time_critical_func(async_uart_check_and_finish_rx)(enum async_uart_rx_err error_code, bool report_if_fifoover) {
    // To avoid any races, all operations clearing rx_active and depending on state data must occur within this critical section
    // Not using a critical section to check/clear rx_active can result in multiple interrupts firing rx_cb
    // Only clearing rx_active in the critical section might allow another transaction to be started in higher priority IRQ
    // and rx_cb or other state data might get overwritten
    uint8_t prev_interrupts = save_and_disable_interrupts();
    __compiler_memory_barrier();
    if (!inst->rx_active) {
        restore_interrupts(prev_interrupts);
        return false;
    }

    // Check for rx overflow
    uint32_t overflow_flag = (1u << PIO_FDEBUG_RXSTALL_LSB) << inst->sm;
    if (report_if_fifoover && (inst->pio->fdebug & overflow_flag)) {
        hw_set_bits(&inst->pio->fdebug, overflow_flag);
        error_code = ASYNC_UART_RX_DATA_LOST;
    }

    // Save the data we need outside crtical section
    async_uart_on_read rx_cb = inst->rx_cb;
    uint8_t *data = NULL;
    size_t len = 0;
    if (error_code == ASYNC_UART_RX_OK) {
        data = inst->rx_data_ptr;
        len = inst->rx_data_len;
    }

    // Clean up the receive state
    irq_set_enabled(ASYNC_UART_DMA_IRQ, false);
    if (dma_channel_is_busy(inst->dma_rx_chan)) {
        dma_channel_abort(inst->dma_rx_chan);
    }
    if (inst->timeout_alarm > 0) {
        cancel_alarm(inst->timeout_alarm);
        inst->timeout_alarm = -1;
    }
    pio_set_irqn_source_enabled(inst->pio, ASYNC_UART_PIO_IRQ_NUM, uart_multidrop_irq_frame_error(inst->sm), false);

    // Mark RX complete
    inst->rx_active = false;
    __compiler_memory_barrier();    // Required to ensure the saved variables won't get optimized after restore_interrupts
    restore_interrupts(prev_interrupts);

    // Now call callback outside critical section
    if (rx_cb)
        rx_cb(error_code, data, len);

    return true;
}

int64_t async_uart_timeout(__unused alarm_id_t id, __unused void *user_data) {
    inst->timeout_alarm = -1;
    async_uart_check_and_finish_rx(ASYNC_UART_RX_TIMEOUT, true);
    return 0;
}

void __time_critical_func(async_uart_dma_handler)(void) {
    dma_irqn_acknowledge_channel(ASYNC_UART_DMA_IRQ_NUM, inst->dma_rx_chan);
    async_uart_check_and_finish_rx(ASYNC_UART_RX_OK, true);
}

void __time_critical_func(async_uart_pio_handler)(void) {
    if (pio_interrupt_get(inst->pio, uart_multidrop_irq_tx_compl_num(inst->sm))) {
        pio_interrupt_clear(inst->pio, uart_multidrop_irq_tx_compl_num(inst->sm));

        // Enter critical section to prevent overwrite of callback function
        uint8_t prev_interrupts = save_and_disable_interrupts();
        __compiler_memory_barrier();
        if (inst->tx_active) {
            // Save callback function for outside critical section
            async_uart_on_write tx_cb = inst->tx_cb;

            // Mark tx complete
            inst->tx_active = false;
            __compiler_memory_barrier();    // Required to ensure the saved variables won't get optimized after restore_interrupts
            restore_interrupts(prev_interrupts);

            if (tx_cb) {
                tx_cb(ASYNC_UART_TX_OK);
            }
        }
        else {
            restore_interrupts(prev_interrupts);
        }
    }

    if (pio_interrupt_get(inst->pio, uart_multidrop_irq_frame_error_num(inst->sm))) {
        pio_interrupt_clear(inst->pio, uart_multidrop_irq_frame_error_num(inst->sm));
        async_uart_check_and_finish_rx(ASYNC_UART_RX_FRAME_ERROR, false);
    }
}

void async_uart_init(PIO pio, unsigned int sm, unsigned int pin, unsigned int baud, unsigned int timeout_ms) {
    inst->pio = pio;
    inst->sm = sm;
    inst->timeout_ms = timeout_ms;

    inst->offset = pio_add_program(pio, &uart_multidrop_program);
    uart_multidrop_program_init(pio, sm, inst->offset, pin, baud);

    inst->dma_tx_chan = dma_claim_unused_channel(true);
    inst->dma_rx_chan = dma_claim_unused_channel(true);

    // Configure tx dma channel
    dma_channel_config tx_c = dma_channel_get_default_config(inst->dma_tx_chan);
    channel_config_set_transfer_data_size(&tx_c, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_write_increment(&tx_c, false);
    channel_config_set_dreq(&tx_c, pio_get_dreq(pio, sm, true));
    dma_channel_configure(
        inst->dma_tx_chan,  // Channel to be configured
        &tx_c,              // The configuration we just created
        &pio->txf[sm],      // The initial write address
        NULL,               // The initial read address
        0,                  // Number of transfers; in this case each is 1 byte.
        false               // Start immediately.
    );

    // Configure tx dma channel
    dma_channel_config rx_c = dma_channel_get_default_config(inst->dma_rx_chan);
    channel_config_set_transfer_data_size(&rx_c, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_c, false);
    channel_config_set_write_increment(&rx_c, true);
    channel_config_set_dreq(&rx_c, pio_get_dreq(pio, sm, false));
    dma_channel_configure(
        inst->dma_rx_chan,  // Channel to be configured
        &rx_c,              // The configuration we just created
        NULL,               // The initial write address
        (void*)(((uintptr_t)&pio->rxf[sm]) + 3),      // The initial read address (upper byte of PIO buffer)
        0,                  // Number of transfers; in this case each is 1 byte.
        false               // Start immediately.
    );

    // Configure IRQ for RX DMA
    dma_irqn_set_channel_enabled(ASYNC_UART_DMA_IRQ_NUM, inst->dma_rx_chan, true);
    irq_set_exclusive_handler(ASYNC_UART_DMA_IRQ, async_uart_dma_handler);

    // Configure IRQs for PIO
    pio_interrupt_clear(pio, uart_multidrop_irq_tx_compl_num(sm));
    pio_set_irqn_source_enabled(pio, ASYNC_UART_PIO_IRQ_NUM, uart_multidrop_irq_tx_compl(sm), true);
    irq_set_exclusive_handler(ASYNC_UQRT_PIO_IRQ(pio), async_uart_pio_handler);
    irq_set_enabled(ASYNC_UQRT_PIO_IRQ(pio), true);
}

void async_uart_write(const uint8_t *data, size_t len, bool preserve_read, async_uart_on_write cb) {
    // Check that no transmit is active in critical section
    uint32_t prev_interrupts = save_and_disable_interrupts();
    if (inst->tx_active) {
        restore_interrupts(prev_interrupts);
        if (cb)
            cb(ASYNC_UART_TX_BUSY);
        return;
    }
    inst->tx_active = true;
    inst->tx_cb = cb;
    restore_interrupts(prev_interrupts);

    pio_sm_set_enabled(inst->pio, inst->sm, false);
    pio_sm_restart(inst->pio, inst->sm);

    // Abort dma channel if in progress
    if (dma_channel_is_busy(inst->dma_tx_chan)) {
        dma_channel_abort(inst->dma_tx_chan);
    }

    if (!preserve_read) {
        // Abort active transfer if pending
        async_uart_check_and_finish_rx(ASYNC_UART_RX_ABORTED, false);

        // Clear receive state
        uint32_t overflow_flag = (1u << PIO_FDEBUG_RXSTALL_LSB) << inst->sm;
        hw_set_bits(&inst->pio->fdebug, overflow_flag);
        pio_sm_clear_fifos(inst->pio, inst->sm);
        pio_interrupt_clear(inst->pio, uart_multidrop_irq_frame_error_num(inst->sm));
    }
    else {
        pio_sm_drain_tx_fifo(inst->pio, inst->sm);
    }

    if (len) {
        // Load x with number of bytes to transmit
        pio_sm_put_blocking(inst->pio, inst->sm, len - 1);
        pio_sm_exec(inst->pio, inst->sm, pio_encode_pull(false, true));
        pio_sm_exec(inst->pio, inst->sm, pio_encode_mov(pio_x, pio_osr));

        // Execute txdata
        pio_sm_exec(inst->pio, inst->sm, pio_encode_jmp(inst->offset + uart_multidrop_offset_txdata));
        pio_interrupt_clear(inst->pio, uart_multidrop_irq_tx_compl_num(inst->sm));
    } else {
        pio_sm_exec(inst->pio, inst->sm, pio_encode_jmp(inst->offset + uart_multidrop_offset_rxdata));
    }

    pio_sm_set_enabled(inst->pio, inst->sm, true);

    if (len) {
        dma_hw->ch[inst->dma_tx_chan].al1_read_addr = (uintptr_t) data;
        dma_hw->ch[inst->dma_tx_chan].al1_transfer_count_trig = len;
    }
    else {
        inst->tx_active = false;
        if (cb)
            cb(ASYNC_UART_TX_OK);
    }
}

void async_uart_read(uint8_t *data, size_t len, async_uart_on_read cb) {
    // Check that no receive is active in critical section

    uint32_t prev_interrupts = save_and_disable_interrupts();
    if (inst->rx_active) {
        restore_interrupts(prev_interrupts);
        if (cb)
            cb(ASYNC_UART_RX_BUSY, NULL, 0);
        return;
    }
    inst->rx_active = true;
    inst->rx_cb = cb;
    inst->rx_data_ptr = data;
    inst->rx_data_len = len;

    // Configure interrupts
    dma_irqn_acknowledge_channel(ASYNC_UART_DMA_IRQ_NUM, inst->dma_rx_chan);
    irq_set_enabled(ASYNC_UART_DMA_IRQ, true);
    inst->timeout_alarm = add_alarm_in_ms(inst->timeout_ms, async_uart_timeout, NULL, false);
    pio_set_irqn_source_enabled(inst->pio, ASYNC_UART_PIO_IRQ_NUM, uart_multidrop_irq_frame_error(inst->sm), true);

    // Begin transfer
    dma_hw->ch[inst->dma_rx_chan].al1_write_addr = (uintptr_t) data;
    dma_hw->ch[inst->dma_rx_chan].al1_transfer_count_trig = len;

    restore_interrupts(prev_interrupts);

    // Call timeout if the timeout alarm didn't schedule
    if (inst->timeout_alarm <= 0) {
        async_uart_check_and_finish_rx(ASYNC_UART_RX_TIMEOUT, true);
    }
}
