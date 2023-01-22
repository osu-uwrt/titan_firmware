#include <stdio.h>
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>

#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

#include "safety/safety.h"

#include "micro_ros_pico/transport_eth.h"

#define UROS_PORT 24321

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* sock */
#define HTTP_sock_MAX_NUM 4

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 1, 43},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 1, 1},                      // Gateway
        .dns = {1, 1, 1, 1},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};


void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(__unused clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

static bool transport_initialized = false;
static uint32_t ip;
static uint16_t port;
static int sock;
bool transport_eth_open(__unused struct uxrCustomTransport * transport)
{
    if (!transport_initialized) {
        transport_initialized = true;

        uint8_t sd, sck_state;

        sd = socket(sock, Sn_MR_UDP, UROS_PORT, SF_IO_NONBLOCK);
        if(sd != sock) {
            //DBG_PRINT(ERROR_DBG, "[%s] sock error\r\n", __func__);
            return false;
        }

        do {
            getsockopt(sd, SO_STATUS, &sck_state);
        } while(sck_state != SOCK_UDP);
    }
    return true;
}

bool transport_eth_close(__unused struct uxrCustomTransport * transport)
{
    return true;
}

size_t transport_eth_write(__unused struct uxrCustomTransport* transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    int snd_len;

    // wiznet sendto will not modify buf
	snd_len = sendto(sock, (uint8_t*) buf, len, (uint8_t *)&ip, port);
	if(snd_len < 0 || (size_t)snd_len != len) {
		//DBG_PRINT(ERROR_DBG, "[%s] sendto error\r\n", __func__);
        *errcode = 1;
		return 0;
	}

	return snd_len;
}

size_t transport_eth_read(__unused struct uxrCustomTransport * transport, uint8_t *buf, size_t len, __unused int timeout, uint8_t *errcode)
{
    int ret;
    int actual_recv_len;
	uint8_t sck_state;
	uint16_t recv_len;

    /*ret = ctlsocket(sock, CS_GET_MAXRXBUF, &recv_max);
    if (ret != SOCK_OK) {
        //DBG_PRINT(ERROR_DBG, "[%s] ctlsocket CS_GET_MAXRXBUF error\r\n", __func__);
        *errcode = 1;
		return 0;
    }*/

	/* Receive Packet Process */
	ret = getsockopt(sock, SO_STATUS, &sck_state);
	if(ret != SOCK_OK) {
		//DBG_PRINT(ERROR_DBG, "[%s] getsockopt SO_STATUS error\r\n", __func__);
        *errcode = 1;
		return 0;
	}

	if(sck_state == SOCK_UDP) {
        /*absolute_time_t timeout_time = make_timeout_time_ms(timeout);
        while (absolute_time_diff_us(timeout_time, get_absolute_time()) < 0) {
            ret = getsockopt(sock, SO_RECVBUF, &recv_len);
            if(ret != SOCK_OK) {
                //DBG_PRINT(ERROR_DBG, "[%s] getsockopt SO_RECVBUF error\r\n", __func__);
                *errcode = 1;
                return 0;
            }
            if (recv_len >= len) {
                break;
            }
            if (recv_len == recv_max) {
                break;
            }
        }*/
        ret = getsockopt(sock, SO_RECVBUF, &recv_len);
        if(ret != SOCK_OK) {
            //DBG_PRINT(ERROR_DBG, "[%s] getsockopt SO_RECVBUF error\r\n", __func__);
            *errcode = 1;
            return 0;
        }

		if(recv_len) {
            uint32_t recv_ip;
            uint16_t recv_port;
			actual_recv_len = recvfrom(sock, buf, len, (uint8_t *)&recv_ip, &recv_port);
			if(actual_recv_len < 0) {
				//DBG_PRINT(ERROR_DBG, "[%s] recvfrom error\r\n", __func__);
                *errcode = 1;
				return 0;
			}

			return actual_recv_len;
		} else {
            return 0;
        }
	} else {
        *errcode = 1;
        return 0;
    }
}

bi_decl(bi_program_feature("Micro-ROS over Ethernet"))

void transport_eth_init(int sock_num, uint32_t target_ip, uint16_t target_port){
    sock = sock_num;
    ip = target_ip;
    port = target_port;

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    network_initialize(g_net_info);

    uint8_t temp = PHY_LINK_OFF;
    printf("Waiting for Ethernet... (Chip ID: 0x%x)\n", getVERSIONR());
    do
    {
        if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
        {
            printf(" Unknown PHY link status\n");

            return;
        }
        safety_tick();

    } while (temp == PHY_LINK_OFF);
    printf("Ethernet Connected!\n");

    rmw_uros_set_custom_transport(
		false,
		NULL,
		transport_eth_open,
		transport_eth_close,
		transport_eth_write,
		transport_eth_read
	);
}