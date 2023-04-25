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

// w5100.h contains private W5x00 hardware "driver" level definitions
// which are not meant to be exposed to other libraries or Arduino users

#ifndef	W5100_H_INCLUDED
#define	W5100_H_INCLUDED

#include "hardware/gpio.h"
#include "hardware/spi.h"

// Safe for all chips
//#define SPI_ETHERNET_SETTINGS SPISettings(14000000, MSBFIRST, SPI_MODE0)

// Safe for W5200 and W5500, but too fast for W5100
// Uncomment this if you know you'll never need W5100 support.
//  Higher SPI clock only results in faster transfer to hosts on a LAN
//  or with very low packet latency.  With ordinary internet latency,
//  the TCP window size & packet loss determine your overall speed.
//#define SPI_ETHERNET_SETTINGS SPISettings(30000000, MSBFIRST, SPI_MODE0)



// Configure the maximum number of sockets to support.  W5100 chips can have
// up to 4 sockets.  W5200 & W5500 can have up to 8 sockets.  Several bytes
// of RAM are used for each socket.  Reducing the maximum can save RAM, but
// you are limited to fewer simultaneous connections.
#if defined(RAMEND) && defined(RAMSTART) && ((RAMEND - RAMSTART) <= 2048)
#define MAX_SOCK_NUM 4
#else
#define MAX_SOCK_NUM 8
#endif

typedef uint8_t SOCKET;

#define SnMR_CLOSE   (uint8_t) 0x00
#define SnMR_TCP     (uint8_t) 0x21
#define SnMR_UDP     (uint8_t) 0x02
#define SnMR_IPRAW   (uint8_t) 0x03
#define SnMR_MACRAW  (uint8_t) 0x04
#define SnMR_PPPOE   (uint8_t) 0x05
#define SnMR_ND      (uint8_t) 0x20
#define SnMR_MULTI   (uint8_t) 0x80

#define SockCMD_OPEN       (uint8_t) 0x01
#define SockCMD_LISTEN     (uint8_t) 0x02
#define SockCMD_CONNECT    (uint8_t) 0x04
#define SockCMD_DISCON     (uint8_t) 0x08
#define SockCMD_CLOSE      (uint8_t) 0x10
#define SockCMD_SEND       (uint8_t) 0x20
#define SockCMD_SEND_MAC   (uint8_t) 0x21
#define SockCMD_SEND_KEEP  (uint8_t) 0x22
#define SockCMD_RECV       (uint8_t) 0x40

#define SnIR_SEND_OK  (uint8_t) 0x10
#define SnIR_TIMEOUT  (uint8_t) 0x08
#define SnIR_RECV     (uint8_t) 0x04
#define SnIR_DISCON   (uint8_t) 0x02
#define SnIR_CON      (uint8_t) 0x01

#define SnSR_CLOSED       (uint8_t) 0x00
#define SnSR_INIT         (uint8_t) 0x13
#define SnSR_LISTEN       (uint8_t) 0x14
#define SnSR_SYNSENT      (uint8_t) 0x15
#define SnSR_SYNRECV      (uint8_t) 0x16
#define SnSR_ESTABLISHED  (uint8_t) 0x17
#define SnSR_FIN_WAIT     (uint8_t) 0x18
#define SnSR_CLOSING      (uint8_t) 0x1A
#define SnSR_TIME_WAIT    (uint8_t) 0x1B
#define SnSR_CLOSE_WAIT   (uint8_t) 0x1C
#define SnSR_LAST_ACK     (uint8_t) 0x1D
#define SnSR_UDP          (uint8_t) 0x22
#define SnSR_IPRAW        (uint8_t) 0x32
#define SnSR_MACRAW       (uint8_t) 0x42
#define SnSR_PPPOE        (uint8_t) 0x5F

#define IPPROTO_IP   (uint8_t) 0
#define IPPROTO_ICMP (uint8_t) 1
#define IPPROTO_IGMP (uint8_t) 2
#define IPPROTO_GGP  (uint8_t) 3
#define IPPROTO_TCP  (uint8_t) 6
#define IPPROTO_PUP  (uint8_t) 12
#define IPPROTO_UDP  (uint8_t) 17
#define IPPROTO_IDP  (uint8_t) 22
#define IPPROTO_ND   (uint8_t) 77
#define IPPROTO_RAW  (uint8_t) 255


enum W5100Linkstatus {
    LINK_UNKNOWN,
    LINK_ON,
    LINK_OFF
};

typedef struct w5k_data {
    spi_inst_t *spi_port;
    uint8_t ss_pin;
    uint8_t chip;           // Chip ID
    uint8_t reset_line;
    uint8_t CH_BASE_MSB;    // Channel base MSB, see W5100_CH_BASE
    uint16_t SSIZE;         // Size of buffer
    uint16_t SMASK;         // Buffer bit mask
    uint16_t local_port;    // The local poart
    struct {
        uint8_t* state_ref; // Pointer to set when socket is in use, clear when not in use
        uint16_t RX_RSR;    // Number of bytes received
        uint16_t RX_RD;     // Address to read
        uint16_t TX_FSR;    // Free space ready for transmit
        uint8_t  RX_inc;    // how much have we advanced RX_RD
    } socket_state[MAX_SOCK_NUM];
} w5k_data_t;

// W5100 Registers
// ---------------
uint16_t w5100_write(w5k_data_t *c, uint16_t addr, const uint8_t *buf, uint16_t len);
static inline uint8_t w5100_write_single(w5k_data_t *c, uint16_t addr, uint8_t data) {
    return w5100_write(c, addr, &data, 1);
}
uint16_t w5100_read(w5k_data_t *c, uint16_t addr, uint8_t *buf, uint16_t len);
static inline uint8_t w5100_read_single(w5k_data_t *c, uint16_t addr) {
    uint8_t data;
    w5100_read(c, addr, &data, 1);
    return data;
}

#define __GP_REGISTER8(name, address)                                            \
static inline void w5100_write##name(w5k_data_t *_c, uint8_t _data) {            \
    w5100_write_single(_c, address, _data);                                      \
  }                                                                              \
static inline uint8_t w5100_read##name(w5k_data_t *_c) {                         \
    return w5100_read_single(_c, address);                                       \
}
#define __GP_REGISTER16(name, address)                                           \
static inline void w5100_write##name(w5k_data_t *_c, uint16_t _data) {           \
    uint8_t buf[2];                                                              \
    buf[0] = _data >> 8;                                                         \
    buf[1] = _data & 0xFF;                                                       \
    w5100_write(_c, address, buf, 2);                                            \
}                                                                                \
static inline uint16_t w5100_read##name(w5k_data_t *_c) {                        \
    uint8_t buf[2];                                                              \
    w5100_read(_c, address, buf, 2);                                             \
    return (buf[0] << 8) | buf[1];                                               \
}
#define __GP_REGISTER_N(name, address, size)                                     \
static inline uint16_t w5100_write##name(w5k_data_t *_c, const uint8_t *_buff) { \
    return w5100_write(_c, address, _buff, size);                                \
}                                                                                \
static inline uint16_t w5100_read##name(w5k_data_t *_c, uint8_t *_buff) {        \
    return w5100_read(_c, address, _buff, size);                                 \
}

__GP_REGISTER8 (MR,     0x0000);    // Mode
__GP_REGISTER_N(GAR,    0x0001, 4); // Gateway IP address
__GP_REGISTER_N(SUBR,   0x0005, 4); // Subnet mask address
__GP_REGISTER_N(SHAR,   0x0009, 6); // Source MAC address
__GP_REGISTER_N(SIPR,   0x000F, 4); // Source IP address
__GP_REGISTER8 (IR,     0x0015);    // Interrupt
__GP_REGISTER8 (IMR,    0x0016);    // Interrupt Mask
__GP_REGISTER16(RTR,    0x0017);    // Timeout address
__GP_REGISTER8 (RCR,    0x0019);    // Retry count
__GP_REGISTER8 (RMSR,   0x001A);    // Receive memory size (W5100 only)
__GP_REGISTER8 (TMSR,   0x001B);    // Transmit memory size (W5100 only)
__GP_REGISTER8 (PATR,   0x001C);    // Authentication type address in PPPoE mode
__GP_REGISTER8 (PTIMER, 0x0028);    // PPP LCP Request Timer
__GP_REGISTER8 (PMAGIC, 0x0029);    // PPP LCP Magic Number
__GP_REGISTER_N(UIPR,   0x002A, 4); // Unreachable IP address in UDP mode (W5100 only)
__GP_REGISTER16(UPORT,  0x002E);    // Unreachable Port address in UDP mode (W5100 only)
__GP_REGISTER8 (VERSIONR_W5200,0x001F);   // Chip Version Register (W5200 only)
__GP_REGISTER8 (VERSIONR_W5500,0x0039);   // Chip Version Register (W5500 only)
__GP_REGISTER8 (PSTATUS_W5200,     0x0035);    // PHY Status
__GP_REGISTER8 (PHYCFGR_W5500,     0x002E);    // PHY Configuration register, default: 10111xxx


#undef __GP_REGISTER8
#undef __GP_REGISTER16
#undef __GP_REGISTER_N

// W5100 Socket registers
// ----------------------
static inline uint16_t W5100_CH_BASE(w5k_data_t *c) {
    //if (chip == 55) return 0x1000;
    //if (chip == 52) return 0x4000;
    //return 0x0400;
    return c->CH_BASE_MSB << 8;
}
static const uint16_t CH_SIZE = 0x0100;

static inline uint8_t w5100_readSn_single(w5k_data_t *c, SOCKET s, uint16_t addr) {
    return w5100_read_single(c, W5100_CH_BASE(c) + s * CH_SIZE + addr);
}
static inline uint8_t w5100_writeSn_single(w5k_data_t *c, SOCKET s, uint16_t addr, uint8_t data) {
    return w5100_write_single(c, W5100_CH_BASE(c) + s * CH_SIZE + addr, data);
}
static inline uint16_t w5100_readSn(w5k_data_t *c, SOCKET s, uint16_t addr, uint8_t *buf, uint16_t len) {
    return w5100_read(c, W5100_CH_BASE(c) + s * CH_SIZE + addr, buf, len);
}
static inline uint16_t w5100_writeSn(w5k_data_t *c, SOCKET s, uint16_t addr, const uint8_t *buf, uint16_t len) {
    return w5100_write(c, W5100_CH_BASE(c) + s * CH_SIZE + addr, buf, len);
}

#define __SOCKET_REGISTER8(name, address)                                 \
static inline void w5100_write##name(w5k_data_t *_c, SOCKET _s, uint8_t _data) {  \
    w5100_writeSn_single(_c, _s, address, _data);                         \
}                                                                         \
static inline uint8_t w5100_read##name(w5k_data_t *_c, SOCKET _s) {               \
    return w5100_readSn_single(_c, _s, address);                          \
}
#define __SOCKET_REGISTER16(name, address)                                \
static inline void w5100_write##name(w5k_data_t *_c, SOCKET _s, uint16_t _data) {        \
    uint8_t buf[2];                                                       \
    buf[0] = _data >> 8;                                                  \
    buf[1] = _data & 0xFF;                                                \
    w5100_writeSn(_c, _s, address, buf, 2);                               \
}                                                                         \
static inline uint16_t w5100_read##name(w5k_data_t *_c, SOCKET _s) {                     \
    uint8_t buf[2];                                                       \
    w5100_readSn(_c, _s, address, buf, 2);                                \
    return (buf[0] << 8) | buf[1];                                        \
}
#define __SOCKET_REGISTER_N(name, address, size)                          \
static inline uint16_t w5100_write##name(w5k_data_t *_c, SOCKET _s, const uint8_t *_buff) {    \
    return w5100_writeSn(_c, _s, address, _buff, size);                   \
}                                                                         \
static inline uint16_t w5100_read##name(w5k_data_t *_c, SOCKET _s, uint8_t *_buff) {     \
    return w5100_readSn(_c, _s, address, _buff, size);                    \
}

__SOCKET_REGISTER8(SnMR,        0x0000)        // Mode
__SOCKET_REGISTER8(SnCR,        0x0001)        // Command
__SOCKET_REGISTER8(SnIR,        0x0002)        // Interrupt
__SOCKET_REGISTER8(SnSR,        0x0003)        // Status
__SOCKET_REGISTER16(SnPORT,     0x0004)        // Source Port
__SOCKET_REGISTER_N(SnDHAR,     0x0006, 6)     // Destination Hardw Addr
__SOCKET_REGISTER_N(SnDIPR,     0x000C, 4)     // Destination IP Addr
__SOCKET_REGISTER16(SnDPORT,    0x0010)        // Destination Port
__SOCKET_REGISTER16(SnMSSR,     0x0012)        // Max Segment Size
__SOCKET_REGISTER8(SnPROTO,     0x0014)        // Protocol in IP RAW Mode
__SOCKET_REGISTER8(SnTOS,       0x0015)        // IP TOS
__SOCKET_REGISTER8(SnTTL,       0x0016)        // IP TTL
__SOCKET_REGISTER8(SnRX_SIZE,   0x001E)        // RX Memory Size (W5200 only)
__SOCKET_REGISTER8(SnTX_SIZE,   0x001F)        // RX Memory Size (W5200 only)
__SOCKET_REGISTER16(SnTX_FSR,   0x0020)        // TX Free Size
__SOCKET_REGISTER16(SnTX_RD,    0x0022)        // TX Read Pointer
__SOCKET_REGISTER16(SnTX_WR,    0x0024)        // TX Write Pointer
__SOCKET_REGISTER16(SnRX_RSR,   0x0026)        // RX Free Size
__SOCKET_REGISTER16(SnRX_RD,    0x0028)        // RX Read Pointer
__SOCKET_REGISTER16(SnRX_WR,    0x002A)        // RX Write Pointer (supported?)

#undef __SOCKET_REGISTER8
#undef __SOCKET_REGISTER16
#undef __SOCKET_REGISTER_N

uint8_t w5100_init(w5k_data_t *c, spi_inst_t *spi, uint8_t ss_pin, uint8_t reset);

static inline void w5100_setGatewayIp(w5k_data_t *c, const uint8_t * addr) { w5100_writeGAR(c, addr); }
static inline void w5100_getGatewayIp(w5k_data_t *c, uint8_t * addr) { w5100_readGAR(c, addr); }

static inline void w5100_setSubnetMask(w5k_data_t *c, const uint8_t * addr) { w5100_writeSUBR(c, addr); }
static inline void w5100_getSubnetMask(w5k_data_t *c, uint8_t * addr) { w5100_readSUBR(c, addr); }

static inline void w5100_setMACAddress(w5k_data_t *c, const uint8_t * addr) { w5100_writeSHAR(c, addr); }
static inline void w5100_getMACAddress(w5k_data_t *c, uint8_t * addr) { w5100_readSHAR(c, addr); }

static inline void w5100_setIPAddress(w5k_data_t *c, const uint8_t * addr) { w5100_writeSIPR(c, addr); }
static inline void w5100_getIPAddress(w5k_data_t *c, uint8_t * addr) { w5100_readSIPR(c, addr); }

static inline void w5100_setRetransmissionTime(w5k_data_t *c, uint16_t timeout) { w5100_writeRTR(c, timeout); }
static inline void w5100_setRetransmissionCount(w5k_data_t *c, uint8_t retry) { w5100_writeRCR(c, retry); }

enum W5100Linkstatus w5100_getLinkStatus(w5k_data_t *c);

void w5100_execCmdSn(w5k_data_t *c, SOCKET s, uint8_t _cmd);

uint8_t w5100_softReset(w5k_data_t *c);
uint8_t w5100_isW5100(w5k_data_t *c);
uint8_t w5100_isW5200(w5k_data_t *c);
uint8_t w5100_isW5500(w5k_data_t *c);

static inline uint8_t w5100_getChip(w5k_data_t *c) { return c->chip; }
static inline uint16_t W5100_SBASE(w5k_data_t *c, uint8_t socknum) {
    if (c->chip == 51) {
        return socknum * c->SSIZE + 0x4000;
    } else {
        return socknum * c->SSIZE + 0x8000;
    }
}
static inline uint16_t W5100_RBASE(w5k_data_t *c, uint8_t socknum) {
    if (c->chip == 51) {
          return socknum * c->SSIZE + 0x6000;
    } else {
        return socknum * c->SSIZE + 0xC000;
    }
}

static inline bool w5100_hasOffsetAddressMapping(w5k_data_t *c) {
    if (c->chip == 55) return true;
    return false;
}

inline static void w5100_initSS(w5k_data_t *c) {
    gpio_set_function(c->ss_pin, GPIO_FUNC_SIO);
    gpio_set_dir(c->ss_pin, GPIO_OUT);
    gpio_put(c->ss_pin, 1);
}
inline static void w5100_setSS(w5k_data_t *c) {
    asm volatile("nop \n nop \n nop");
    gpio_put(c->ss_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}
inline static void w5100_resetSS(w5k_data_t *c) {
    asm volatile("nop \n nop \n nop");
    gpio_put(c->ss_pin, 1);
    asm volatile("nop \n nop \n nop");
}



#endif

#ifndef UTIL_H
#define UTIL_H

#ifndef htons
// The host order of the Arduino platform is little endian.
// Sometimes it is desired to convert to big endian (or
// network order)

// Host to Network short
#define htons(x) ( (((x)&0xFF)<<8) | (((x)>>8)&0xFF) )

// Network to Host short
#define ntohs(x) htons(x)

// Host to Network long
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )

// Network to Host long
#define ntohl(x) htonl(x)

#endif // !defined(htons)

#endif
