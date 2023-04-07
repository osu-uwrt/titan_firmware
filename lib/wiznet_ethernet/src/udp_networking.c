#include "eth_networking.h"

bool eth_udp_begin(udp_socket_t *s, w5k_data_t *c, uint16_t port)
{
    s->socket_active = 0;
	s->sockindex = eth_socketBegin(c, SnMR_UDP, port, &s->socket_active);
	if (s->sockindex >= MAX_SOCK_NUM){
        //printf("Socket index invalid: %d\n", s->sockindex);

        return false;
    }
    s->device = c;
	s->port = port;
	s->remaining = 0;
    s->offset = 0;
    s->remoteIP[0] = 0;
    s->remoteIP[1] = 0;
    s->remoteIP[2] = 0;
    s->remoteIP[3] = 0;
    s->remotePort = 0;
	return true;
}

/* return number of bytes available in the current packet,
   will return zero if parsePacket hasn't been called yet */
size_t eth_udp_available(udp_socket_t *s)
{
    if (!s->socket_active){
        return 0;
    }
	return s->remaining;
}

/* Release any resources being used by this EthernetUDP instance */
void eth_udp_stop(udp_socket_t *s)
{
	if (s->socket_active) {
		eth_close_socket(s->device, s->sockindex);
	}
}

bool eth_udp_isopen(udp_socket_t *s)
{
	return s->socket_active;
}

int eth_udp_beginPacket(udp_socket_t *s, uint8_t *ip, uint16_t port)
{
    if (!s->socket_active){
        return 0;
    }
	s->offset = 0;
	//printf("UDP beginPacket\n");
	return eth_socketStartUDP(s->device, s->sockindex, ip, port);
}

int eth_udp_endPacket(udp_socket_t *s)
{
    if (!s->socket_active){
        return 0;
    }
	return eth_socketSendUDP(s->device, s->sockindex);
}

size_t eth_udp_write_single(udp_socket_t *s, uint8_t byte)
{
	return eth_udp_write(s, &byte, 1);
}

size_t eth_udp_write(udp_socket_t *s, const uint8_t *buffer, size_t size)
{
    if (!s->socket_active){
        return 0;
    }
	//printf("UDP write %d\n", size);
	uint16_t bytes_written = eth_socketBufferData(s->device, s->sockindex, s->offset, buffer, size);
	s->offset += bytes_written;
	return bytes_written;
}

size_t eth_udp_parsePacket(udp_socket_t *s)
{
    if (!s->socket_active) {
        return 0;
    }
	// discard any remaining bytes in the last packet
	while (s->remaining) {
		// could this fail (loop endlessly) if _remaining > 0 and recv in read fails?
		// should only occur if recv fails after telling us the data is there, lets
		// hope the w5100 always behaves :)
		eth_udp_read_single(s);
	}

	if (eth_socketRecvAvailable(s->device, s->sockindex) > 0) {
		//HACK - hand-parse the UDP packet using TCP recv method
		uint8_t tmpBuf[8];
		int ret=0;
		//read 8 header bytes and get IP and port from it
		ret = eth_socketRecv(s->device, s->sockindex, tmpBuf, 8);
		if (ret > 0) {
            for (int i = 0; i < 4; i++){
                s->remoteIP[i] = tmpBuf[i];
            }
			s->remotePort = tmpBuf[4];
			s->remotePort = (s->remotePort << 8) + tmpBuf[5];
			s->remaining = tmpBuf[6];
			s->remaining = (s->remaining << 8) + tmpBuf[7];

			// When we get here, any remaining bytes are the data
			ret = s->remaining;
		}
		return ret;
	}
	// There aren't any packets available
	return 0;
}

uint8_t eth_udp_read_single(udp_socket_t *s)
{
	uint8_t byte;

	if ((s->remaining > 0) && s->socket_active && (eth_socketRecv(s->device, s->sockindex, &byte, 1) > 0)) {
		// We read things without any problems
		s->remaining--;
		return byte;
	}

	// If we get here, there's no data available
	return -1;
}

size_t eth_udp_read(udp_socket_t *s, unsigned char *buffer, size_t len)
{
	if (s->remaining > 0 && s->socket_active) {
		int got;
		if (s->remaining <= len) {
			// data should fit in the buffer
			got = eth_socketRecv(s->device, s->sockindex, buffer, s->remaining);
		} else {
			// too much data for the buffer,
			// grab as much as will fit
			got = eth_socketRecv(s->device, s->sockindex, buffer, len);
		}
		if (got > 0) {
			s->remaining -= got;
			//printf("UDP read %d\n", got);
			return got;
		}
	}
	// If we get here, there's no data available or recv failed
	return -1;
}

uint8_t eth_udp_peek(udp_socket_t *s)
{
	// Unlike recv, peek doesn't check to see if there's any data available, so we must.
	// If the user hasn't called parsePacket yet then return nothing otherwise they
	// may get the UDP header
	if (!s->socket_active || s->remaining == 0) return -1;
	return eth_socketPeek(s->device, s->sockindex);
}
