#ifndef PIO_SMBUS_H_
#define PIO_SMBUS_H_

#include "i2c.pio.h"

// PIO Command was successful
#define PIO_SMBUS_SUCCESS 0
// The Address byte was NAKed (this means the device is offline)
#define PIO_SMBUS_ERR_ADDR_NAK -1
// The Address byte was NAKed after restart (the device responded once before, but didn't ACK it's addr after restart?)
#define PIO_SMBUS_ERR_ADDR_RESTART_NAK -2
// A data byte was NAKed (this means the device did respond, but had an error)
#define PIO_SMBUS_ERR_NAK -3
// The clock was stretched beyond an SMBus timeout
#define PIO_SMBUS_ERR_TIMEOUT -4
// Either SDA or SCL was low before sending a START. The bus must be idle before calling a function
#define PIO_SMBUS_ERR_BUS_STUCK_LOW -5
// The transmitted data did not match the state of the bus (someone else was holding it low)
#define PIO_SMBUS_ERR_ABRITRATION_LOST -6
// The SMBus Packet Error Checking value did not match the expected value
#define PIO_SMBUS_ERR_BAD_CHECKSUM -7
// The SMBus slave sent more bytes than was able to be received into the provided buffer
#define PIO_SMBUS_ERR_BUF_TOO_SMALL -8
// Attempted to perform an SMBus write larger than the maximum supported transmit length
#define PIO_SMBUS_ERR_BUF_TOO_LARGE -9
// The response received from the device is invalid (such as receiving a response of the wrong length)
// This isn't used explicitly by the driver, but can be used by wrappers of this driver to report decode errors
#define PIO_SMBUS_ERR_INVALID_RESP -10

// All of these functions below follow the SMBus specification
// See http://smbus.org/specs/smbus110.pdf for more information
// The PIO and SM point to the SM configured to run SMBus
// The addr is the SMBus address, cmd is the SMBus command
// The _pec variant enable Packet Error Checking (PEC). This must be supported by the slave

int pio_smbus_word_read(PIO pio, uint sm, uint8_t addr, uint8_t cmd);
int pio_smbus_word_read_pec(PIO pio, uint sm, uint8_t addr, uint8_t cmd);
int pio_smbus_word_write(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint16_t data);
int pio_smbus_word_write_pec(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint16_t data);
int pio_smbus_block_read(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *rxbuf, uint max_len);
int pio_smbus_block_read_pec(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *rxbuf, uint max_len);
int pio_smbus_block_write(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *txbuf, uint len);
int pio_smbus_block_write_pec(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *txbuf, uint len);

#endif
