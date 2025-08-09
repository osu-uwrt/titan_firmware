#include "ivc.h"

#include "fft/fft.h"
#include "ros.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <math.h>

// General defines
#define FREQ_LOW_HZ 17000
#define FREQ_HIGH_HZ 19000
#define SYSCLK_HZ clock_get_hz(clk_sys)
#define SYMBOL_PERIOD_MS 50

#define CRC_POLY 0x1B  // 0b11011 (x^4 + x^3 + x + 1)
#define DATA_SIZE 8
#define CRC_SIZE 4
#define PACKET_SIZE (DATA_SIZE + CRC_SIZE)  // 1-byte MTU + 4-bit CRC
#define MAX_MISSED_PACKETS 5

#define CALIBRATION_MODE 0

// Tx data
// Define wrap such that clkdiv is on [0, 256)
#define PWM_WRAP_VALUE ((int) (((float) SYSCLK_HZ) / 5000 / 255.0f) + 1.0f)
#define HEARTBEAT_PERIOD_MS 5000
#define HEARTBEAT_DATA 0x3F
static uint pwm_slice_num;
static int tx_data_idx;
static uint16_t tx_data;
static repeating_timer_t tx_timer = { 0 };
static bool last_ack_good = false;
static uint8_t link_established = 0;
static bool is_talos;
static absolute_time_t next_heartbeat_time = { 0 };
static absolute_time_t ssfb_tx_time_bound = { 0 };

#define TX_QUEUE_DEPTH 8
typedef struct tx_msg_t {
    uint8_t data;
    bool is_heartbeat;
} tx_msg;
static struct QUEUE_DEFINE(tx_msg, TX_QUEUE_DEPTH) tx_queue = { 0 };

// Rx data
// #define NUM_SAMPLES 500
// #define ADC_FREQ_HZ 500000.0f
// #define TIMESTEP_S 1.0f / ADC_FREQ_HZ
#define NUM_BINS 2
#define BIN_RANGE 500

#define SSFB_FREQ_LOW_THRESHOLD 300
#define SSFB_FREQ_HIGH_THRESHOLD 300
#define TALOS_FREQ_LOW_THRESHOLD 300
#define TALOS_FREQ_HIGH_THRESHOLD 300

const int FREQ_LOW_THRESHOLD[] = { SSFB_FREQ_LOW_THRESHOLD, TALOS_FREQ_LOW_THRESHOLD };
const int FREQ_HIGH_THRESHOLD[] = { SSFB_FREQ_HIGH_THRESHOLD, TALOS_FREQ_HIGH_THRESHOLD };
// float freqs[NUM_BINS] = { 18000.0f, 19000.0f };
// static int dma_chan;

frequency_bin_t bins[NUM_BINS] = { { "freq_low", FREQ_LOW_HZ - BIN_RANGE, FREQ_LOW_HZ + BIN_RANGE, 0 },
                                   { "freq_high", FREQ_HIGH_HZ - BIN_RANGE, FREQ_HIGH_HZ + BIN_RANGE, 0 } };

uint8_t sample_bufs[2][NSAMP];
bool buf_select = 0;
int fft_target = -1;
// float mags[NUM_BINS];
// uint64_t last_dma_time = { 0 };
// uint64_t last_dma_period = { 0 };
bool packet_in_flight = false;
bool rx_packet_in_flight = false;
absolute_time_t next_rx_time = { 0 };
uint8_t last_rx_val = -1;
repeating_timer_t rx_timer = { 0 };
uint8_t rx_idx = 0;
uint16_t rx_packet;

uint8_t rx_arr[PACKET_SIZE + 1];

void ivc_tx(uint16_t data);

// Calculate CRC-4
static uint8_t calculate_crc(uint8_t packet) {
    // uint16_t packet_agumented = ((uint16_t) packet) << CRC_SIZE;
    uint8_t crc = 0;
    for (int i = DATA_SIZE - 1; i >= 0; i--) {
        crc <<= 1;
        crc |= (packet >> i) & 0x01;

        if (crc & 0x10)
            crc ^= CRC_POLY;
    }

    return crc & 0x0F;  // Only care about lower 4 bits; note: this will need changed if CRC_SIZE is different
}

static int rx_single_sample() {
    fft_process(sample_bufs[fft_target], bins, NUM_BINS);

    int val = -1;

    if (bins[0].amplitude > FREQ_LOW_THRESHOLD[is_talos] && bins[1].amplitude < FREQ_HIGH_THRESHOLD[is_talos])
        val = 0;
    if (bins[1].amplitude > FREQ_HIGH_THRESHOLD[is_talos] && bins[0].amplitude < FREQ_LOW_THRESHOLD[is_talos])
        val = 1;

    return val;
}

static int64_t interframe_space_cb(__unused alarm_id_t id, __unused void *user_data) {
    if (last_ack_good) {
        link_established = MAX_MISSED_PACKETS;
        packet_in_flight = false;
    }
    else if (link_established) {
        link_established--;
        // ivc_tx(tx_data >> CRC_SIZE);
        ivc_tx(tx_data);
    }
    else {
        LOG_WARN("IVC link broken!");
        packet_in_flight = false;
    }

    return 0;
}

static int64_t ack_read(__unused alarm_id_t id, __unused void *user_data) {
    last_ack_good = (rx_single_sample() == 1);
    add_alarm_in_ms((PACKET_SIZE + 1) * SYMBOL_PERIOD_MS, interframe_space_cb, NULL, true);

    LOG_INFO("Got response of %d from receiver", last_ack_good);

    return 0;
}

// Note: the caller is responsible for enabling and disabling the transducer to use this function
static void tx_set_bitwise(int bit) {
    float freq = (bit) ? FREQ_HIGH_HZ : FREQ_LOW_HZ;
    pwm_set_clkdiv(pwm_slice_num, clock_get_hz(clk_sys) / (freq * PWM_WRAP_VALUE));
}

static bool tx_cb(__unused repeating_timer_t *rt) {
    if (tx_data_idx < 0) {
        pwm_set_enabled(pwm_slice_num, false);
        // packet_in_flight = false;
        add_alarm_in_ms(1.5f * SYMBOL_PERIOD_MS, ack_read, NULL, true);
        return false;
    }

    // float freq = (tx_data >> tx_data_idx) & 0x01 ? FREQ_HIGH_HZ : FREQ_LOW_HZ;
    // pwm_set_clkdiv(pwm_slice_num, clock_get_hz(clk_sys) / (freq * PWM_WRAP_VALUE));
    tx_set_bitwise((tx_data >> tx_data_idx) & 0x01);

    tx_data_idx--;
    return true;
}

// This function requires data to be pre-shifted left by CRC_SIZE
void ivc_tx(uint16_t data) {
    tx_data_idx = PACKET_SIZE;  // Transmit one byte's worth of data + CRC + heartbeat flag
    // tx_data = ((uint16_t) data) << CRC_SIZE;
    tx_data = data;
    tx_data |= calculate_crc(data >> CRC_SIZE);  // Shift the data back to LSB

    packet_in_flight = true;
    pwm_set_enabled(pwm_slice_num, true);

    // Indicate start of packet with high frequency
    // pwm_set_clkdiv(pwm_slice_num, clock_get_hz(clk_sys) / (FREQ_HIGH_HZ * PWM_WRAP_VALUE));
    tx_set_bitwise(0x01);

    add_repeating_timer_ms(-SYMBOL_PERIOD_MS, tx_cb, NULL, &tx_timer);
}

void ivc_enqueue_packet(uint8_t data, bool is_heartbeat) {
    if (QUEUE_FULL(&tx_queue))
        return;

    tx_msg *msg = QUEUE_CUR_WRITE_ENTRY(&tx_queue);
    msg->data = data;
    msg->is_heartbeat = is_heartbeat;
    QUEUE_MARK_WRITE_DONE(&tx_queue);
}

static bool ivc_dequeue_packet(uint8_t *data, bool *is_heartbeat) {
    if (QUEUE_EMPTY(&tx_queue))
        return false;

    tx_msg *msg = QUEUE_CUR_READ_ENTRY(&tx_queue);
    *data = msg->data;
    *is_heartbeat = msg->is_heartbeat;
    QUEUE_MARK_READ_DONE(&tx_queue);

    return true;
}

static void tx_init() {
    // Find out which PWM slice is connected to TX_PIN (it's slice 0)
    gpio_set_function(TX_PIN, GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(TX_PIN);
    uint pwm_chan = pwm_gpio_to_channel(TX_PIN);

    pwm_set_wrap(pwm_slice_num, PWM_WRAP_VALUE - 1);
    pwm_set_chan_level(pwm_slice_num, pwm_chan, PWM_WRAP_VALUE / 2);

    // Select lpf output
    gpio_init(OUTPUT_SELECT_PIN);
    gpio_set_dir(OUTPUT_SELECT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_SELECT_PIN, 1);
}

static void sample_handler() {
    //     uint64_t now = to_us_since_boot(get_absolute_time());
    //     last_dma_period = now - last_dma_time;
    //     last_dma_time = now;

    //     // Start DMA running on the other buffer
    //     buf_select = !buf_select;
    //     dma_channel_set_write_addr(dma_chan, &sample_bufs[buf_select], true);

    //     // Call Goertzel's on the previously-collected sample
    //     goertzel_target = !buf_select;

    // // Clear the interrupt request
    // dma_hw->ints0 = 1u << dma_chan;

    fft_target = buf_select;
    buf_select = !buf_select;
    fft_sample(sample_bufs[buf_select]);

    // fft_sample(sample_bufs[0]);

    // fft_process(sample_bufs[!buf_select], bins, NUM_BINS);

    // int val = -1;

    // if (bins[0].amplitude > FREQ_LOW_THRESHOLD[is_talos] && bins[1].amplitude < FREQ_HIGH_THRESHOLD[is_talos])
    //     val = 0;
    // if (bins[1].amplitude > FREQ_HIGH_THRESHOLD[is_talos] && bins[0].amplitude < FREQ_LOW_THRESHOLD[is_talos])
    //     val = 1;

    // LOG_INFO("Got IVC value as %d", val);
}

static int64_t ack_send_cb(__unused alarm_id_t id, __unused void *user_data) {
    pwm_set_enabled(pwm_slice_num, false);
    ssfb_tx_time_bound = make_timeout_time_ms(DATA_SIZE * SYMBOL_PERIOD_MS);
    rx_packet_in_flight = false;

    return 0;
}

static bool rx_cb(__unused repeating_timer_t *rt) {
    if (rx_idx >= PACKET_SIZE + 1) {                    // PACKET_SIZE + 1 to include the heartbeat bit
        uint8_t data = (rx_packet >> CRC_SIZE) & 0xFF;  // Remove heartbeat bit
        uint8_t crc = rx_packet & 0x0F;                 // Note: this will need changed if CRC_SIZE is different

        // Send ACK or NACK based on crc

        bool crc_good = crc == calculate_crc(data);

        pwm_set_enabled(pwm_slice_num, true);
        tx_set_bitwise(crc_good & 0x01);
        add_alarm_in_ms(SYMBOL_PERIOD_MS, ack_send_cb, NULL, true);

        if (!crc_good) {
            LOG_INFO("Got CRC_BAD: got %hhu and expected %hhu", crc, calculate_crc(data));
            return false;
        }

        link_established = MAX_MISSED_PACKETS;
        LOG_INFO("Got CRC_GOOD");

        if (rx_packet & (1 << PACKET_SIZE)) {
            // Heartbeat bit is set. Ignore this packet
            LOG_INFO("Got heartbeat");
            return false;
        }

        ros_publish_rx(data);

        // LOG_INFO("Sending ACK as %d", crc_good & 0x01);

        // LOG_INFO("Got IVC packet %hhd", data);
        // for (int i = 0; i < PACKET_SIZE + 1; i++) {
        //     LOG_INFO("%hhu", rx_arr[i]);
        // }
        // for (int i = 7; i >= 0; i--) {
        //     LOG_INFO("%d", (data >> i) & 0x01);
        // }

        return false;
    }

    int val = rx_single_sample();

    rx_packet |= val << (PACKET_SIZE - rx_idx);
    rx_arr[rx_idx] = val;
    rx_idx++;

    return true;
}

void ivc_tick() {
    // if (goertzel_target != -1) {
    //     goertzel(goertzel_target);

    //     // for (int i = 0; i < NUM_BINS; i++) {
    //     //     LOG_INFO("Got mag %f for frequency %f Hz\n", mags[i], freqs[i]);
    //     // }
    //     LOG_INFO("%0.2f: %f, %0.2f: %f", freqs[0], mags[0], freqs[1], mags[1]);
    //     // LOG_INFO("%f", mags[0]);
    //     // LOG_INFO("\n");
    // }

    // LOG_INFO("FIFO at: %hhu", adc_fifo_get_level());

    if (fft_target != -1 && !rx_packet_in_flight && !packet_in_flight) {
        int val = rx_single_sample();
        last_rx_val = val;

        // if (bins[0].amplitude > FREQ_LOW_THRESHOLD[is_talos] && bins[1].amplitude >
        // FREQ_HIGH_THRESHOLD[is_talos])
        //     // LOG_INFO("Both mags above threshold. Rejecting!");
        //     val = 2;

        // if (rx_packet_in_flight && time_reached(next_rx_time)) {
        //     if (rx_idx < 8) {
        //         next_rx_time = make_timeout_time_ms(SYMBOL_PERIOD_MS);
        //         rx_packet |= val << (7 - rx_idx);
        //         rx_arr[rx_idx] = val;
        //         rx_idx++;
        //         // LOG_INFO("Got IVC bit %d with mags %f, %f", val, bins[0].amplitude, bins[1].amplitude);
        //     }
        //     else {
        //         LOG_INFO("Got IVC packet %hhd", (int8_t) rx_packet);
        //         for (int i = 0; i < 8; i++) {
        //             LOG_INFO("%hhu", rx_arr[i]);
        //         }
        //         rx_packet_in_flight = false;
        //     }
        // }

#if CALIBRATION_MODE
        ros_publish_rx_debug(bins[0].amplitude);
#else
        if (!rx_packet_in_flight && val == 1) {
            rx_packet_in_flight = true;
            rx_idx = rx_packet = 0;
            // next_rx_time = make_timeout_time_ms(0.0f * SYMBOL_PERIOD_MS);
            sleep_ms(0.3f * SYMBOL_PERIOD_MS);
            add_repeating_timer_ms(-SYMBOL_PERIOD_MS, rx_cb, NULL, &rx_timer);
        }
#endif

        // LOG_INFO("Got IVC value as %d", val);

        // printf("%s: Amplitude = %f\n", bins[1].name, bins[1].amplitude);
        // printf("%f, %f\n", bins[0].amplitude, bins[1].amplitude);

        // printf("%d\n", bins[0].amplitude > 1000);

        fft_target = -1;
    }

    // Only talos sends out heartbeat msgs and awaits response from SSFB
    if (is_talos && time_reached(next_heartbeat_time)) {
        ivc_enqueue_packet(HEARTBEAT_DATA, true);
        next_heartbeat_time = make_timeout_time_ms(HEARTBEAT_PERIOD_MS);
    }

    uint8_t tx_packet;
    bool is_heartbeat;
    if (!packet_in_flight && !rx_packet_in_flight && (is_talos || !time_reached(ssfb_tx_time_bound)) &&
        ivc_dequeue_packet(&tx_packet, &is_heartbeat)) {
        // SSFB can only send messages for a pre-determined period after hearing one from Talos
        ivc_tx((tx_packet << CRC_SIZE) | (is_heartbeat << PACKET_SIZE));
        // ivc_tx(tx_packet << CRC_SIZE);
    }
}

static void rx_init() {
    // Config ADC
    // adc_gpio_init(RX_PIN);
    // adc_init();
    // adc_select_input(RX_PIN - 26);
    // adc_fifo_setup(true,   // Write each completed conversion to the sample FIFO
    //                true,   // Enable DMA data request (DREQ)
    //                1,      // DREQ (and IRQ) asserted when at least 1 sample present
    //                false,  // We won't see the ERR bit because of 8 bit reads; disable.
    //                true    // Shift each sample to 8 bits when pushing to FIFO
    // );

    // *** Don't set clkdiv since we want the ADC running in continuous mode
    // adc_set_clkdiv(ADC_BASE_FREQ / (ADC_FREQ_HZ * ADC_NUM_SAMPLE_CYCLES));
    // adc_set_clkdiv(1088);

    // Config DMA
    // dma_chan = dma_claim_unused_channel(false);
    // if (dma_chan == -1) {
    //     LOG_ERROR("\n\nDIDN'T GET A DMA CHANNEL\n\n");
    // }

    // dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    // channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    // channel_config_set_read_increment(&cfg, false);
    // channel_config_set_write_increment(&cfg, true);
    // channel_config_set_dreq(&cfg, DREQ_ADC);

    // dma_channel_configure(dma_chan, &cfg, &sample_bufs[0], &adc_hw->fifo, NUM_SAMPLES, false);  // Don't start
    // yet

    // dma_channel_set_irq0_enabled(dma_chan, true);  // Set IRQ
    // irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    // irq_set_enabled(DMA_IRQ_0, true);

    // // Now start the DMA
    // dma_channel_start(dma_chan);
    // adc_run(true);

    fft_setup(sample_handler);
    fft_sample(sample_bufs[0]);
}

void ivc_init() {
    tx_init();
    rx_init();

    gpio_init(BOARD_ID_PIN);
    gpio_set_dir(BOARD_ID_PIN, GPIO_IN);

    is_talos = !gpio_get(BOARD_ID_PIN);
    LOG_INFO("This board %s talos", is_talos ? "is" : "is not");
}
