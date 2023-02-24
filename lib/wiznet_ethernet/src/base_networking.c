/*
 * Copyright 2021 Robert Pafford
 * Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "eth_networking.h"

// No yield on bare metal RP2040
#define yield()

// Initializes a new W5K compatible network device with the specified SPI configuration and mac address
// Returns either a pointer to the w5k data struct or NULL of the initialization failed.
w5k_data_t eth_init(spi_inst_t *spi, uint8_t ss_pin, uint8_t reset, uint8_t *mac)
{
    w5k_data_t c = (w5k_data_t)malloc(sizeof(struct w5k_data));
	if (c == NULL){
		return NULL;
	}
	if (w5100_init(c, spi, ss_pin, reset) == 0) {
        free((void*)c);
        return NULL;
    }
    w5100_setMACAddress(c, mac);
    return c;
}

// Configures the W5K device with the specified network configuration
//
void eth_ifconfig(w5k_data_t c, uint8_t* ip, uint8_t* gateway, uint8_t* subnet)
{
	w5100_setIPAddress(c, ip);
	w5100_setGatewayIp(c, gateway);
	w5100_setSubnetMask(c, subnet);
}

// De-initializes the socket. Note when this is called it frees the w5k data pointer
// Do not call any functions after this function is called using the w5k data pointer
void eth_deinit(w5k_data_t c)
{
    w5100_softReset(c);
    // TODO: Reset SS pin state
    for (int i = 0; i < MAX_SOCK_NUM; i++){
        if (c->socket_state[i].state_ref != NULL){
            *c->socket_state[i].state_ref = 0;
        }
    }
    free(c);
}

/*****************************************/
/*      Socket Management Functions      */
/*****************************************/

// Opens a socket with the specified protocol and port. state_ref will be set to 1 when socket is open
// and will be set to zero when socket is closed. Returns the socket number. If no sockets are available
// MAX_SOCK_NUMBER will be returned.
SOCKET eth_socketBegin(w5k_data_t c, uint8_t protocol, uint16_t port, uint8_t *state_ref)
{
    if (state_ref != NULL) {
        *state_ref = 0;
    }
	uint8_t s, status[MAX_SOCK_NUM], chip, maxindex=MAX_SOCK_NUM;

	// first check hardware compatibility
	chip = w5100_getChip(c);
	if (!chip) return MAX_SOCK_NUM; // immediate error if no hardware detected
#if MAX_SOCK_NUM > 4
	if (chip == 51) maxindex = 4; // W5100 chip never supports more than 4 sockets
#endif
	//printf("W5000socket begin, protocol=%d, port=%d\n", protocol, port);
	// look at all the hardware sockets, use any that are closed (unused)
	for (s=0; s < maxindex; s++) {
		status[s] = w5100_readSnSR(c, s);
		if (status[s] == SnSR_CLOSED) goto makesocket;
	}
	//printf("W5000socket step2\n");
	// as a last resort, forcibly close any already closing
	for (s=0; s < maxindex; s++) {
		uint8_t stat = status[s];
		if (stat == SnSR_LAST_ACK) goto closemakesocket;
		if (stat == SnSR_TIME_WAIT) goto closemakesocket;
		if (stat == SnSR_FIN_WAIT) goto closemakesocket;
		if (stat == SnSR_CLOSING) goto closemakesocket;
	}
#if 0
	printf("W5000socket step3\n");
	// next, use any that are effectively closed
	for (s=0; s < MAX_SOCK_NUM; s++) {
		uint8_t stat = status[s];
		// TODO: this also needs to check if no more data
		if (stat == SnSR::CLOSE_WAIT) goto closemakesocket;
	}
#endif
	return MAX_SOCK_NUM; // all sockets are in use
closemakesocket:
	//printf("W5000socket close\n");
	w5100_execCmdSn(c, s, SockCMD_CLOSE);
    if (c->socket_state[s].state_ref != NULL) {
        *c->socket_state[s].state_ref = 0;
        c->socket_state[s].state_ref = NULL;
    }
makesocket:
	//printf("W5000socket %d\n", s);
	busy_wait_ms(250); // TODO: is this needed??
	w5100_writeSnMR(c, s, protocol);
	w5100_writeSnIR(c, s, 0xFF);
	if (port > 0) {
		w5100_writeSnPORT(c, s, port);
	} else {
		// if don't set the source port, set local_port number.
		if (++c->local_port < 49152) c->local_port = 49152;
		w5100_writeSnPORT(c, s, c->local_port);
	}
	w5100_execCmdSn(c, s, SockCMD_OPEN);
	c->socket_state[s].RX_RSR = 0;
	c->socket_state[s].RX_RD  = w5100_readSnRX_RD(c, s); // always zero?
	c->socket_state[s].RX_inc = 0;
	c->socket_state[s].TX_FSR = 0;
    c->socket_state[s].state_ref = state_ref;
    if (state_ref != NULL){
        *state_ref = 1;
    }
	//printf("W5000socket prot=%d, RX_RD=%d\n", W5100.readSnMR(s), state[s].RX_RD);
	return s;
}

SOCKET eth_socketCreate(w5k_data_t c, uint8_t* ip, uint16_t sport, uint8_t protocol, uint16_t dport, uint8_t *state_ref)
{
	SOCKET sock = eth_socketBegin(c, protocol, sport, state_ref);
	w5100_writeSnDIPR(c, sock, ip);
	w5100_writeSnDPORT(c, sock, dport);
	return sock;
}

// Closes the socket. If there are any active users of the socket it will deinit that as well
//
void eth_close_socket(w5k_data_t c, SOCKET s)
{
    if (c->socket_state[s].state_ref != NULL) {
        *c->socket_state[s].state_ref = 0;
        c->socket_state[s].state_ref = NULL;
    }
	w5100_execCmdSn(c, s, SockCMD_CLOSE);
}

// Get socket status. Returns the enum of the current socket status
//
uint8_t eth_socketStatus(w5k_data_t c, SOCKET s)
{
	uint8_t status = w5100_readSnSR(c, s);
	return status;
}

/*****************************************/
/*    Socket Data Receive Functions      */
/*****************************************/


static uint16_t getSnRX_RSR(w5k_data_t c, uint8_t s)
{
#if 1
        uint16_t val, prev;

        prev = w5100_readSnRX_RSR(c, s);
        while (1) {
                val = w5100_readSnRX_RSR(c, s);
                if (val == prev) {
			return val;
		}
                prev = val;
        }
#else
	uint16_t val = W5100.readSnRX_RSR(s);
	return val;
#endif
}

static void read_data(w5k_data_t c, uint8_t s, uint16_t src, uint8_t *dst, uint16_t len)
{
	uint16_t size;
	uint16_t src_mask;
	uint16_t src_ptr;

	//printf("read_data, len=%d, at:%d\n", len, src);
	src_mask = (uint16_t)src & c->SMASK;
	src_ptr = W5100_RBASE(c, s) + src_mask;

	if (w5100_hasOffsetAddressMapping(c) || src_mask + len <= c->SSIZE) {
		w5100_read(c, src_ptr, dst, len);
	} else {
		size = c->SSIZE - src_mask;
		w5100_read(c, src_ptr, dst, size);
		dst += size;
		w5100_read(c, W5100_RBASE(c, s), dst, len - size);
	}
}

// Receive data.  Returns size, or -1 for no data, or 0 if connection closed
//
int eth_socketRecv(w5k_data_t c, uint8_t s, uint8_t *buf, int16_t len)
{
	// Check how much data is available
	int ret = c->socket_state[s].RX_RSR;
	if (ret < len) {
		uint16_t rsr = getSnRX_RSR(c, s);
		ret = rsr - c->socket_state[s].RX_inc;
		c->socket_state[s].RX_RSR = ret;
		//printf("Sock_RECV, RX_RSR=%d, RX_inc=%d\n", ret, state[s].RX_inc);
	}
	if (ret == 0) {
		// No data available.
		uint8_t status = w5100_readSnSR(c, s);
		if ( status == SnSR_LISTEN || status == SnSR_CLOSED ||
		  status == SnSR_CLOSE_WAIT ) {
			// The remote end has closed its side of the connection,
			// so this is the eof state
			ret = 0;
		} else {
			// The connection is still up, but there's no data waiting to be read
			ret = -1;
		}
	} else {
		if (ret > len) ret = len; // more data available than buffer length
		uint16_t ptr = c->socket_state[s].RX_RD;
		if (buf) read_data(c, s, ptr, buf, ret);
		ptr += ret;
		c->socket_state[s].RX_RD = ptr;
		c->socket_state[s].RX_RSR -= ret;
		uint16_t inc = c->socket_state[s].RX_inc + ret;
		if (inc >= 250 || c->socket_state[s].RX_RSR == 0) {
			c->socket_state[s].RX_inc = 0;
			w5100_writeSnRX_RD(c, s, ptr);
			w5100_execCmdSn(c, s, SockCMD_RECV);
			//printf("Sock_RECV cmd, RX_RD=%d, RX_RSR=%d\n",
			//  state[s].RX_RD, state[s].RX_RSR);
		} else {
			c->socket_state[s].RX_inc = inc;
		}
	}
	//printf("socketRecv, ret=%d\n", ret);
	return ret;
}

uint16_t eth_socketRecvAvailable(w5k_data_t c, uint8_t s)
{
	uint16_t ret = c->socket_state[s].RX_RSR;
	if (ret == 0) {
		uint16_t rsr = getSnRX_RSR(c, s);
		ret = rsr - c->socket_state[s].RX_inc;
		c->socket_state[s].RX_RSR = ret;
		//printf("sockRecvAvailable s=%d, RX_RSR=%d\n", s, ret);
	}
	return ret;
}

// get the first byte in the receive queue (no checking)
//
uint8_t eth_socketPeek(w5k_data_t c, uint8_t s)
{
	uint8_t b;
	uint16_t ptr = c->socket_state[s].RX_RD;
	w5100_read(c, (ptr & c->SMASK) + W5100_RBASE(c, s), &b, 1);
	return b;
}



/*****************************************/
/*    Socket Data Transmit Functions     */
/*****************************************/

static uint16_t getSnTX_FSR(w5k_data_t c, uint8_t s)
{
        uint16_t val, prev;

        prev = w5100_readSnTX_FSR(c, s);
        while (1) {
                val = w5100_readSnTX_FSR(c, s);
                if (val == prev) {
			c->socket_state[s].TX_FSR = val;
			return val;
		}
                prev = val;
        }
}


static void write_data(w5k_data_t c, uint8_t s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
	uint16_t ptr = w5100_readSnTX_WR(c, s);
	ptr += data_offset;
	uint16_t offset = ptr & c->SMASK;
	uint16_t dstAddr = offset + W5100_SBASE(c, s);

	if (w5100_hasOffsetAddressMapping(c) || offset + len <= c->SSIZE) {
		w5100_write(c, dstAddr, data, len);
	} else {
		// Wrap around circular buffer
		uint16_t size = c->SSIZE - offset;
		w5100_write(c, dstAddr, data, size);
		w5100_write(c, W5100_SBASE(c, s), data + size, len - size);
	}
	ptr += len;
	w5100_writeSnTX_WR(c, s, ptr);
}


/**
 * @brief	This function used to send the data in TCP mode
 * @return	1 for success else 0.
 */
uint16_t eth_socketSend(w5k_data_t c, uint8_t s, const uint8_t * buf, uint16_t len)
{
	uint8_t status=0;
	uint16_t ret=0;
	uint16_t freesize=0;

	if (len > c->SSIZE) {
		ret = c->SSIZE; // check size not to exceed MAX size.
	} else {
		ret = len;
	}

	// if freebuf is available, start.
	do {
		freesize = getSnTX_FSR(c, s);
		status = w5100_readSnSR(c, s);
		if ((status != SnSR_ESTABLISHED) && (status != SnSR_CLOSE_WAIT)) {
			ret = 0;
			break;
		}
		yield();
	} while (freesize < ret);

	// copy data
	write_data(c, s, 0, (uint8_t *)buf, ret);
	w5100_execCmdSn(c, s, SockCMD_SEND);

	/* +2008.01 bj */
	while ( (w5100_readSnIR(c, s) & SnIR_SEND_OK) != SnIR_SEND_OK ) {
		/* m2008.01 [bj] : reduce code */
		if ( w5100_readSnSR(c, s) == SnSR_CLOSED ) {
			return 0;
		}
		yield();
	}
	/* +2008.01 bj */
	w5100_writeSnIR(c, s, SnIR_SEND_OK);
	return ret;
}

uint16_t eth_socketSendAvailable(w5k_data_t c, uint8_t s)
{
	uint8_t status=0;
	uint16_t freesize=0;
	freesize = getSnTX_FSR(c, s);
	status = w5100_readSnSR(c, s);
	if ((status == SnSR_ESTABLISHED) || (status == SnSR_CLOSE_WAIT)) {
		return freesize;
	}
	return 0;
}

uint16_t eth_socketBufferData(w5k_data_t c, uint8_t s, uint16_t offset, const uint8_t* buf, uint16_t len)
{
	//printf("  bufferData, offset=%d, len=%d\n", offset, len);
	uint16_t ret =0;
	uint16_t txfree = getSnTX_FSR(c, s);
	if (len > txfree) {
		ret = txfree; // check size not to exceed MAX size.
	} else {
		ret = len;
	}
	write_data(c, s, offset, buf, ret);
	return ret;
}

bool eth_socketStartUDP(w5k_data_t c, uint8_t s, uint8_t* addr, uint16_t port)
{
	if ( ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
	  ((port == 0x00)) ) {
		return false;
	}
	w5100_writeSnDIPR(c, s, addr);
	w5100_writeSnDPORT(c, s, port);
	return true;
}

bool eth_socketSendUDP(w5k_data_t c, uint8_t s)
{
	w5100_execCmdSn(c, s, SockCMD_SEND);

	/* +2008.01 bj */
	while ( (w5100_readSnIR(c, s) & SnIR_SEND_OK) != SnIR_SEND_OK ) {
		if (w5100_readSnIR(c, s) & SnIR_TIMEOUT) {
			/* +2008.01 [bj]: clear interrupt */
			w5100_writeSnIR(c, s, (SnIR_SEND_OK|SnIR_TIMEOUT));
			//printf("sendUDP timeout\n");
			return false;
		}
		yield();
	}

	/* +2008.01 bj */
	w5100_writeSnIR(c, s, SnIR_SEND_OK);

	//printf("sendUDP ok\n");
	/* Sent ok */
	return true;
}
