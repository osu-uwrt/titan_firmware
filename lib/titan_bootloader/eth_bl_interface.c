#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/flash.h"

#include "driver/wiznet.h"
#include "titan/canmore.h"
#include "bl_interface.h"

typedef uint8_t IPAddress[4];
static uint8_t mac[] = {0x2A, 0xCD, 0x00, 0x00, 0x00, 0x00};
static uint8_t device_ip[] = ETHERNET_IP;
static IPAddress gateway = ETHERNET_GATEWAY;
static IPAddress subnet = ETHERNET_MASK;

static IPAddress heartbeat_broadcast = CANMORE_TITAN_ETH_BROADCAST_IP;
static uint16_t heartbeat_port = CANMORE_TITAN_ETH_HEARTBEAT_BROADCAST_PORT;
static uint16_t control_port = CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT;

static w5k_data_t eth_device;
static udp_socket_t heartbeat_socket;
static udp_socket_t control_socket;

bool bl_interface_init(void) {
    // Initialize MAC address with lower values in Flash ID to *randomize* it sort of
    uint8_t uniqueId[8];
    flash_get_unique_id(uniqueId);
    mac[2] = uniqueId[4];
    mac[3] = uniqueId[5];
    mac[4] = uniqueId[6];
    mac[5] = uniqueId[7];

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(ETH_SPI ? spi1 : spi0, 14*1000*1000);
    spi_set_format( ETH_SPI ? spi1 : spi0,   // SPI instance
                    8,      // Number of bits per transfer
                    0,      // Polarity (CPOL)
                    0,      // Phase (CPHA)
                    SPI_MSB_FIRST);
    gpio_set_function(ETH_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ETH_CLK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(ETH_MOSI_PIN, GPIO_FUNC_SPI);

    // set the gpio functions for cs and rst
    gpio_init(ETH_CS_PIN);
    gpio_init(ETH_RST_PIN);
    gpio_set_dir(ETH_RST_PIN, GPIO_OUT);
    gpio_set_dir(ETH_CS_PIN, GPIO_OUT);

    // set cs to de-assert
    gpio_put(ETH_CS_PIN, 1);

    //reset routine
    gpio_put(ETH_RST_PIN, 0);
    busy_wait_ms(1);
    gpio_put(ETH_RST_PIN, 1);
    busy_wait_ms(150);

    if (!eth_init(&eth_device, ETH_SPI ? spi1 : spi0, ETH_CS_PIN, ETH_RST_PIN, mac)){
        return false;
    }

    // Configure RTR and RCR to not insane values
    w5100_setRetransmissionTime(&eth_device, 300);	// 30ms timeout
	w5100_setRetransmissionCount(&eth_device, 1);

    // start the Ethernet
    eth_ifconfig(&eth_device, device_ip, gateway, subnet);

    // Verify IP configuration
    IPAddress tmp;
    bool mismatch = false;
    w5100_getIPAddress(&eth_device, tmp);
    for(size_t i = 0; i < 4; i++){
        if(tmp[i] != device_ip[i])
            mismatch = true;
    }
    if(mismatch) return false;

    // Begin UDP sockets
    if (!eth_udp_begin(&heartbeat_socket, &eth_device, heartbeat_port)) {
        return false;
    }
    if (!eth_udp_begin(&control_socket, &eth_device, control_port)) {
        return false;
    }

    return true;
}

static void bl_interface_send_heartbeat_byte(uint8_t mode) {
    static uint8_t cnt = 0;
    uint8_t byte = CANMORE_CALC_TITAN_HEARTBEAT_DATA(cnt, 0, mode, 0, 0);

    cnt++;
    if (cnt >= (1<<CANMORE_HEARTBEAT_CNT_LENGTH)) {
        cnt = 0;
    }

    if (!eth_udp_beginPacket(&heartbeat_socket, heartbeat_broadcast, heartbeat_port)) {
        __breakpoint();
		return;
	}

	if (eth_udp_write(&heartbeat_socket, &byte, 1) != 1) {
        __breakpoint();
        return;
	}

	if (!eth_udp_endPacket(&heartbeat_socket)) {
        __breakpoint();
    }
}

void bl_interface_heartbeat(void) {
    bl_interface_send_heartbeat_byte(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER);
}

void bl_interface_notify_boot(void) {
    busy_wait_ms(1000);
    bl_interface_send_heartbeat_byte(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOT_DELAY);
}

bool bl_interface_check_online(void){
    return w5100_getLinkStatus(&eth_device) == LINK_ON;
}

bool bl_interface_try_receive(uint8_t *msg_out, size_t *len_out) {
    // make sure the socket is still open
    if (!eth_udp_isopen(&control_socket)){
		return false;
	}

    // need to parse the packet first and see if we have data
    size_t packetSize = eth_udp_parsePacket(&control_socket);
	if (!packetSize){
        return false;
	}

    // make sure the packet isnt too big
	if (BL_INTERFACE_MAX_PACKET_LEN < packetSize){
		return false;
	}

    // now we can attempt the socket reads
	if (eth_udp_read(&control_socket, msg_out, BL_INTERFACE_MAX_PACKET_LEN) != packetSize) {
		return false;
	}

    *len_out = packetSize;
    return true;
}

void bl_interface_transmit(uint8_t *msg, size_t len) {
    if (!eth_udp_beginPacket(&control_socket, eth_udp_remoteIP(&control_socket), eth_udp_remotePort(&control_socket))) {
		return;
	}

	if (eth_udp_write(&control_socket, msg, len) != len) {
        return;
	}

	eth_udp_endPacket(&control_socket);
}
