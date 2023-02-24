/*
 * Copyright 2021 Robert Pafford
 * Copyright 2018 Paul Stoffregen
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <stdio.h>
#include <stdlib.h>
#include "w5100.h"
#include "random.h"

uint8_t w5100_init(w5k_data_t c, spi_inst_t *spi, uint8_t ss_pin, uint8_t reset)
{
	seed_random_from_rosc();

	c->spi_port = spi;
	c->ss_pin = ss_pin;
	c->reset_line = reset;
	c->SSIZE = 2048;
	c->SMASK = 0x07FF;
	c->chip = 0;
	c->local_port = random_range(49152, 65535);
	for (int i = 0; i < MAX_SOCK_NUM; i++){
		c->socket_state[i].state_ref = NULL;
	}

	static bool initialized = false;
	uint8_t i;

	if (initialized) return 1;

	//puts("w5100 init");

	w5100_initSS(c);
	w5100_resetSS(c);

	// Attempt W5200 detection first, because W5200 does not properly
	// reset its SPI state when CS goes high (inactive).  Communication
	// from detecting the other chips can leave the W5200 in a state
	// where it won't recover, unless given a reset pulse.
	if (w5100_isW5200(c)) {
		c->CH_BASE_MSB = 0x40;
#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
		c->SSIZE = 16384;
#elif MAX_SOCK_NUM <= 2
		c->SSIZE = 8192;
#elif MAX_SOCK_NUM <= 4
		c->SSIZE = 4096;
#else
		c->SSIZE = 2048;
#endif
		c->SMASK = c->SSIZE - 1;
#endif
		for (i=0; i<MAX_SOCK_NUM; i++) {
			w5100_writeSnRX_SIZE(c, i, c->SSIZE >> 10);
			w5100_writeSnTX_SIZE(c, i, c->SSIZE >> 10);
		}
		for (; i<8; i++) {
			w5100_writeSnRX_SIZE(c, i, 0);
			w5100_writeSnTX_SIZE(c, i, 0);
		}
	// Try W5500 next.  WIZnet finally seems to have implemented
	// SPI well with this chip.  It appears to be very resilient,
	// so try it after the fragile W5200
	} else if (w5100_isW5500(c)) {
		c->CH_BASE_MSB = 0x10;
#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
		c->SSIZE = 16384;
#elif MAX_SOCK_NUM <= 2
		c->SSIZE = 8192;
#elif MAX_SOCK_NUM <= 4
		c->SSIZE = 4096;
#else
		c->SSIZE = 2048;
#endif
		c->SMASK = c->SSIZE - 1;
		for (i=0; i<MAX_SOCK_NUM; i++) {
			w5100_writeSnRX_SIZE(c, i, c->SSIZE >> 10);
			w5100_writeSnTX_SIZE(c, i, c->SSIZE >> 10);
		}
		for (; i<8; i++) {
			w5100_writeSnRX_SIZE(c, i, 0);
			w5100_writeSnTX_SIZE(c, i, 0);
		}
#endif
	// Try W5100 last.  This simple chip uses fixed 4 byte frames
	// for every 8 bit access.  Terribly inefficient, but so simple
	// it recovers from "hearing" unsuccessful W5100 or W5200
	// communication.  W5100 is also the only chip without a VERSIONR
	// register for identification, so we check this last.
	} else if (w5100_isW5100(c)) {
		c->CH_BASE_MSB = 0x04;
#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
		c->SSIZE = 8192;
		w5100_writeTMSR(c, 0x03);
		w5100_writeRMSR(c, 0x03);
#elif MAX_SOCK_NUM <= 2
		c->SSIZE = 4096;
		w5100_writeTMSR(c, 0x0A);
		w5100_writeRMSR(c, 0x0A);
#else
		c->SSIZE = 2048;
		w5100_writeTMSR(c, 0x55);
		w5100_writeRMSR(c, 0x55);
#endif
		c->SMASK = c->SSIZE - 1;
#else
		w5100_writeTMSR(c, 0x55);
		w5100_writeRMSR(c, 0x55);
#endif
	// No hardware seems to be present.  Or it could be a W5200
	// that's heard other SPI communication if its chip select
	// pin wasn't high when a SD card or other SPI chip was used.
	} else {
		//puts("no chip :-(");
		c->chip = 0;
		return 0; // no known chip is responding :-(
	}
	initialized = true;
	return 1; // successful init
}

// Soft reset the WIZnet chip, by writing to its MR register reset bit
uint8_t w5100_softReset(w5k_data_t c)
{
	uint16_t count=0;

	//puts("WIZnet soft reset");
	// write to reset bit
	w5100_writeMR(c, 0x80);
	// then wait for soft reset to complete
	do {
		uint8_t mr = w5100_readMR(c);
		//printf("mr=%02X\n", mr);
		if (mr == 0) {
			return 1;
		} else if (mr == 255) {
			// Someone fucked up to get here
    		gpio_put(c->reset_line, 0);
    		busy_wait_ms(1500);
    		gpio_put(c->reset_line, 1);
    		busy_wait_ms(100);
			w5100_writeMR(c, 0x80);
		}
		busy_wait_ms(5);
	} while (++count < 20);
	return 0;
}

uint8_t w5100_isW5100(w5k_data_t c)
{
	c->chip = 51;
	//puts("w5100.cpp: detect W5100 chip");
	if (!w5100_softReset(c)) return 0;
	w5100_writeMR(c, 0x10);
	if (w5100_readMR(c) != 0x10) return 0;
	w5100_writeMR(c, 0x12);
	if (w5100_readMR(c) != 0x12) return 0;
	w5100_writeMR(c, 0x00);
	if (w5100_readMR(c) != 0x00) return 0;
	//puts("chip is W5100");
	return 1;
}

uint8_t w5100_isW5200(w5k_data_t c)
{
	c->chip = 52;
	//puts("w5100.cpp: detect W5200 chip");
	if (!w5100_softReset(c)) return 0;
	//w5100_writeMR(c, 0x08);
	//if (w5100_readMR(c) != 0x08) return 0;
	w5100_writeMR(c, 0x10);
	if (w5100_readMR(c) != 0x10) return 0;
	w5100_writeMR(c, 0x00);
	if (w5100_readMR(c) != 0x00) return 0;
	int ver = w5100_readVERSIONR_W5200(c);
	//printf("version=%d\n");
	if (ver != 3) return 0;
	//puts("chip is W5200");
	return 1;
}

uint8_t w5100_isW5500(w5k_data_t c)
{
	c->chip = 55;
	//puts("w5100.cpp: detect W5500 chip");
	if (!w5100_softReset(c)) return 0;
	w5100_writeMR(c, 0x08);
	if (w5100_readMR(c) != 0x08) return 0;
	w5100_writeMR(c, 0x10);
	if (w5100_readMR(c) != 0x10) return 0;
	w5100_writeMR(c, 0x00);
	if (w5100_readMR(c) != 0x00) return 0;
	int ver = w5100_readVERSIONR_W5500(c);
	//printf("version=%d\n",ver);
	if (ver != 4) return 0;
	//puts("chip is W5500");
	return 1;
}

enum W5100Linkstatus w5100_getLinkStatus(w5k_data_t c)
{
	uint8_t phystatus;

	// if (!w5100_init(c, c->spi_port, c->ss_pin, c->reset_line)) return LINK_UNKNOWN;
	switch (c->chip) {
	  case 52:
		phystatus = w5100_readPSTATUS_W5200(c);
		if (phystatus & 0x20) return LINK_ON;
		return LINK_OFF;
	  case 55:
		phystatus = w5100_readPHYCFGR_W5500(c);
		if (phystatus & 0x01) return LINK_ON;
		return LINK_OFF;
	  default:
		return LINK_UNKNOWN;
	}
}

uint16_t w5100_read(w5k_data_t c, uint16_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t cmd[4];

	if (c->chip == 51) {
		for (uint16_t i=0; i < len; i++) {
			w5100_setSS(c);
			cmd[0] = 0x0F;
			cmd[1] = addr >> 8;
			cmd[2] = addr & 0xFF;
			cmd[3] = 0;
			spi_write_blocking(c->spi_port, cmd, 4); // TODO: why doesn't this work?
			buf[i] = cmd[3];
			addr++;
			w5100_resetSS(c);
		}
	} else if (c->chip == 52) {
		w5100_setSS(c);
		cmd[0] = addr >> 8;
		cmd[1] = addr & 0xFF;
		cmd[2] = (len >> 8) & 0x7F;
		cmd[3] = len & 0xFF;
		spi_write_blocking(c->spi_port, cmd, 4);
		spi_read_blocking(c->spi_port, 0, buf, len);
		w5100_resetSS(c);
	} else { // chip == 55
		w5100_setSS(c);
		if (addr < 0x100) {
			// common registers 00nn
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = 0x00;
		} else if (addr < 0x8000) {
			// socket registers  10nn, 11nn, 12nn, 13nn, etc
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = ((addr >> 3) & 0xE0) | 0x08;
		} else if (addr < 0xC000) {
			// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
			//  10## #nnn nnnn nnnn
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x10;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x10; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x10; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x10; // 2K buffers
			#endif
		} else {
			// receive buffers
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x18;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x18; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x18; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x18; // 2K buffers
			#endif
		}
		spi_write_blocking(c->spi_port, cmd, 3);
		spi_read_blocking(c->spi_port, 0, buf, len);
		w5100_resetSS(c);
	}
	return len;
}

uint16_t w5100_write(w5k_data_t c, uint16_t addr, const uint8_t *buf, uint16_t len)
{
	uint8_t cmd[8];

	if (c->chip == 51) {
		for (uint16_t i=0; i<len; i++) {
			w5100_setSS(c);
			uint8_t data = 0xF0;
			spi_write_blocking(c->spi_port, &data, 1);
			data = addr >> 8;
			spi_write_blocking(c->spi_port, &data, 1);
			data = addr & 0xFF;
			spi_write_blocking(c->spi_port, &data, 1);
			addr++;
			spi_write_blocking(c->spi_port, &buf[i], 1);
			w5100_resetSS(c);
		}
	} else if (c->chip == 52) {
		w5100_setSS(c);
		cmd[0] = addr >> 8;
		cmd[1] = addr & 0xFF;
		cmd[2] = ((len >> 8) & 0x7F) | 0x80;
		cmd[3] = len & 0xFF;
		spi_write_blocking(c->spi_port, cmd, 4);
		spi_write_blocking(c->spi_port, buf, len);
		w5100_resetSS(c);
	} else { // chip == 55
		w5100_setSS(c);
		if (addr < 0x100) {
			// common registers 00nn
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = 0x04;
		} else if (addr < 0x8000) {
			// socket registers  10nn, 11nn, 12nn, 13nn, etc
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = ((addr >> 3) & 0xE0) | 0x0C;
		} else if (addr < 0xC000) {
			// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
			//  10## #nnn nnnn nnnn
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x14;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x14; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x14; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x14; // 2K buffers
			#endif
		} else {
			// receive buffers
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x1C;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x1C; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x1C; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x1C; // 2K buffers
			#endif
		}
		if (len <= 5) {
			for (uint8_t i=0; i < len; i++) {
				cmd[i + 3] = buf[i];
			}
			spi_write_blocking(c->spi_port, cmd, len + 3);
		} else {
			spi_write_blocking(c->spi_port, cmd, 3);
			spi_write_blocking(c->spi_port, buf, len);
		}
		w5100_resetSS(c);
	}
	return len;
}

void w5100_execCmdSn(w5k_data_t c, SOCKET s, uint8_t _cmd)
{
	// Send command to socket
	w5100_writeSnCR(c, s, _cmd);
	// Wait for command to complete
	while (w5100_readSnCR(c, s)) ;
}
