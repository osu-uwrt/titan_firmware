#include "ledc_io.h"

#include "ledc_start.h"

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "titan/logger.h"

/**
 * @brief decodes a read spi packet into an integer representation
 *
 * @param in_packet the packet (array of 4 bytes)
 *
 * @return the integer representation of the packet
 */
static spi_frame_t decode_packet(uint8_t in_packet[]) {
    spi_frame_t frame = 0;
    frame |= in_packet[0];
    frame <<= 8;
    frame |= in_packet[1];
    frame <<= 8;
    frame |= in_packet[2];
    frame <<= 8;
    frame |= in_packet[3];
    return frame;
}

/**
 * @brief encodes a spi packet to be written IN PLACE
 *
 * @param out_packet the packet to be written (array of 4 bytes)
 * @param opcode the opcode to be encoded in upper 2 bits of byte 4
 * @param address the address we are writing to, encoded in lower 6 bits of byte 4
 * @param data the data to be encoded into the lower 3 bytes of the packet
 */
static void encode_packet(uint8_t out_packet[], uint8_t opcode, uint8_t address, uint32_t data) {
    if (address <= MAX_ADDRESS) {
        out_packet[0] = (uint8_t) ((opcode << OPCODE_SHIFT) | (address & 0x3F));
        out_packet[1] = (uint8_t) ((data >> 16) & (0xFF));
        out_packet[2] = (uint8_t) ((data >> 8) & (0xFF));
        out_packet[3] = (uint8_t) (data & (0xFF));
    }
    if (!good_parity(out_packet))
        out_packet[3] |= 0b1;
}

/**
 * @brief performs a spi transfer (parallel write/read) to one of the led controllers.
 *
 * @param target_chip the controller we are talking to
 * @param opcode the opcode we are using
 * @param address the address we are writing to
 * @param data the data we are writing
 *
 * @return an integer representation of the data read from the controller after writing
 */
static spi_frame_t spi_transfer(uint8_t target_chip, uint8_t opcode, uint8_t address, uint32_t data) {
    uint8_t out_packet[SPI_PACKET_SIZE];
    uint8_t in_packet[SPI_PACKET_SIZE];

    encode_packet(out_packet, opcode, address, data);

    gpio_put(target_chip, 0);
    busy_wait_us(1);
    if (spi_write_read_blocking(LEDC_SPI_INST, out_packet, in_packet, SPI_PACKET_SIZE) != SPI_PACKET_SIZE) {
        LOG_ERROR("ERROR: couldn't transfer over spi\n");
    }
    busy_wait_us(1);
    gpio_put(target_chip, 1);

    return decode_packet(in_packet);
}

uint8_t get_global_status(spi_frame_t frame) {
    uint8_t gsb = frame >> 24;  // global status byte
    bool limp_home = gsb & 1;
    bool global_warning = gsb & (1 << 1);
    bool pwm_clk_fail = gsb & (1 << 2);
    bool critical_err = gsb & (1 << 3);
    bool non_critical_err = gsb & (1 << 4);
    bool spi_err = gsb & (1 << 5);
    bool reset = gsb & (1 << 6);

    if (limp_home) {
        LOG_ERROR("ERROR: device in limp home\n");
    }
    return gsb;
}

bool good_parity(uint8_t raw_frame[]) {
    uint8_t ones_count = 0;
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t piece = raw_frame[i];
        while (piece) {
            ones_count += piece & 1;
            piece >>= 1;
        }
    }
    return ones_count % 2 == 1;
}

spi_frame_t spi_write(uint8_t target_chip, uint8_t address, uint32_t data) {
    uint8_t target_cs_pin = target_chip == LEDC1 ? LEDC_NCS1_PIN : LEDC_NCS2_PIN;
    return spi_transfer(target_cs_pin, OPCODE_WRITE, address, data);
}

spi_frame_t spi_read(uint8_t target_chip, uint8_t address) {
    uint8_t target_cs_pin = target_chip == LEDC1 ? LEDC_NCS1_PIN : LEDC_NCS2_PIN;
    return spi_transfer(target_cs_pin, OPCODE_READ, address, NULL);
}

spi_frame_t spi_read_clr(uint8_t target_chip, uint8_t address) {
    uint8_t target_cs_pin = target_chip == LEDC1 ? LEDC_NCS1_PIN : LEDC_NCS2_PIN;
    return spi_transfer(target_cs_pin, OPCODE_READ_AND_CLEAR, address, NULL);
}

spi_frame_t spi_read_dev_info(uint8_t target_chip, uint8_t address) {
    uint8_t target_cs_pin = target_chip == LEDC1 ? LEDC_NCS1_PIN : LEDC_NCS2_PIN;
    return spi_transfer(target_cs_pin, OPCODE_READ_DEVICE_INFO, address, NULL);
}
