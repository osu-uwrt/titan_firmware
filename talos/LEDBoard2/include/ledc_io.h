#ifndef LEDC_IO_H
#define LEDC_IO_H

#include <stdbool.h>
#include <stdint.h>

#define OPCODE_WRITE 0
#define OPCODE_READ 1
#define OPCODE_READ_AND_CLEAR 2
#define OPCODE_READ_DEVICE_INFO 3

#define LEDC1 0
#define LEDC2 1

// each LEDC has 2 bucks
#define BUCK1 0
#define BUCK2 1

#define MAX_ADDRESS 63
#define OPCODE_SHIFT 6
#define SPI_PACKET_SIZE 4

/**
 * 32 bit unsigned integer type constructed from a buffer
 * that read data is put in.
 */
typedef uint32_t spi_frame_t;

/**
 * @brief gets the global status byte from a spi read
 *
 * @param frame the spi frame that was read
 *
 * @return the global status byte
 */
uint8_t get_global_status(spi_frame_t frame);

/**
 * @brief ensures the integrity of a spi frame. Data is valid if parity is odd
 *
 * @param raw_frame spi frame to check before writing
 *
 * @return true if valid, else false
 */
bool good_parity(uint8_t raw_frame[]);

/**
 * @brief write data to a desired led controller at a specific address
 *
 * @param target_chip led controller we want to write to
 * @param address the address we are writing to
 * @param data the data we are writing
 *
 * @return integer representation of data read after writing
 */
spi_frame_t spi_write(uint8_t target_chip, uint8_t address, uint32_t data);

/**
 * @brief read data from an address on a specific led controller
 *
 * @param target_chip the desired led controller
 * @param address the address we want to read from
 *
 * @return integer representation of the data read
 */
spi_frame_t spi_read(uint8_t target_chip, uint8_t address);

/**
 * @brief specific read an clear command, issued to a certain led controller
 *
 * @param target_chip the desired led controller
 * @param address the address we want to read and clear from
 *
 * @return integer representation of the data read
 */
spi_frame_t spi_read_clr(uint8_t target_chip, uint8_t address);

/**
 * @brief specific read device info command to read from ROM on a controller
 *
 * @param target_chip the desired led controller
 * @param address address we want to read from
 *
 * @return integer representation of the data read
 */
spi_frame_t spi_read_dev_info(uint8_t target_chip, uint8_t address);

#endif  // LEDC_IO_H
