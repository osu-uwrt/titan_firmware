#ifndef DRIVER__WIZNET__ETH_NETWORKING_H_
#define DRIVER__WIZNET__ETH_NETWORKING_H_

#include <stdbool.h>
#include <stdint.h>
#include "driver/wiznet/w5100.h"
#include "hardware/spi.h"

// Network Card Functions

bool eth_init(w5k_data_t *cOut, spi_inst_t *spi, uint8_t ss_pin, uint8_t reset, uint8_t *mac);
void eth_ifconfig(w5k_data_t *c, const uint8_t* ip, const uint8_t* gateway, const uint8_t* subnet);
void eth_deinit(w5k_data_t *c);
SOCKET eth_socketBegin(w5k_data_t *c, uint8_t protocol, uint16_t port, uint8_t *state_ref);
SOCKET eth_socketCreate(w5k_data_t *c, uint8_t* ip, uint16_t sport, uint8_t protocol, uint16_t dport, uint8_t *state_ref);
void eth_close_socket(w5k_data_t *c, SOCKET s);
uint8_t eth_socketStatus(w5k_data_t *c, SOCKET s);
int eth_socketRecv(w5k_data_t *c, uint8_t s, uint8_t *buf, int16_t len);
uint16_t eth_socketRecvAvailable(w5k_data_t *c, uint8_t s);
uint8_t eth_socketPeek(w5k_data_t *c, uint8_t s);
uint16_t eth_socketSend(w5k_data_t *c, uint8_t s, const uint8_t * buf, uint16_t len);
uint16_t eth_socketSendAvailable(w5k_data_t *c, uint8_t s);
uint16_t eth_socketBufferData(w5k_data_t *c, uint8_t s, uint16_t offset, const uint8_t* buf, uint16_t len);
bool eth_socketStartUDP(w5k_data_t *c, uint8_t s, const uint8_t* addr, uint16_t port);
bool eth_socketSendUDP(w5k_data_t *c, uint8_t s);


// UDP Socket Functions

typedef struct udp_socket {
    w5k_data_t *device;      // The w5k data struct
    SOCKET sockindex;        // The socket index in the device
    uint8_t socket_active;   // If the socket is active on the device (Cleared by lower level logic). Note: if this is zero after init, w5k is no longer valid
    uint16_t port;           // Local port to listen on
    uint16_t remaining;      // Remaining bytes of incoming packet yet to be processed
    uint16_t offset;         // Offset into the packet being sent
    uint8_t remoteIP[4];     // Remote IP address for the incoming packet whilst it's being processed
    uint16_t remotePort;     // Remote port for the incoming packet whilst it's being processed
} udp_socket_t;

bool eth_udp_begin(udp_socket_t *socketOut, w5k_data_t *c, uint16_t port);
size_t eth_udp_available(udp_socket_t *s);
void eth_udp_stop(udp_socket_t *s);
bool eth_udp_isopen(udp_socket_t *s);
int eth_udp_beginPacket(udp_socket_t *s, const uint8_t *ip, uint16_t port);
int eth_udp_endPacket(udp_socket_t *s);
size_t eth_udp_write_single(udp_socket_t *s, uint8_t byte);
size_t eth_udp_write(udp_socket_t *s, const uint8_t *buffer, size_t size);
size_t eth_udp_parsePacket(udp_socket_t *s);
uint8_t eth_udp_read_single(udp_socket_t *s);
size_t eth_udp_read(udp_socket_t *s, unsigned char *buffer, size_t len);
uint8_t eth_udp_peek(udp_socket_t *s);

static inline uint8_t* eth_udp_remoteIP(udp_socket_t *s) { return s->remoteIP; }
static inline uint16_t eth_udp_remotePort(udp_socket_t *s) { return s->remotePort; }

#endif