#include <stdio.h>
#include "pico/binary_info.h"
#include "pico/unique_id.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>

#include "safety/safety.h"
#include "eth_networking.h"

#include "micro_ros_pico/transport_eth.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

typedef uint8_t IPAddress[4];

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Communication Definitions */
// The ROBOT_ variables are defined in the robot header file defined by UWRT_ROBOT
// If these are giving not defined errors, ensure that UWRT_ROBOT is defined in the CMakeLists file for the project
static uint8_t dest_ip[] = ROBOT_COMPUTER_IP;
static uint16_t dest_port = ROBOT_COMPUTER_UROS_PORT;
static uint8_t source_ip[] = ETHERNET_IP;
static uint16_t source_port = ETHERNET_PORT;
static IPAddress gateway = ETHERNET_GATEWAY;
static IPAddress subnet = ETHERNET_MASK;
static uint8_t mac[] = {0x2A, 0xCD, 0xC1, 0x12, 0x34, 0x56};
static udp_socket_t ros_socket;
static w5k_data_t eth_device;
static bool showed_closed = false;


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
bool transport_eth_open(__unused struct uxrCustomTransport * transport)
{
    if (!transport_initialized && ros_socket.socket_active) {
        transport_initialized = true;
    }
    return true;
}

bool transport_eth_close(__unused struct uxrCustomTransport * transport)
{
    // eth_udp_stop(ros_socket);
    return true;
}

size_t transport_eth_write(__unused struct uxrCustomTransport* transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{

    if (!eth_udp_beginPacket(&ros_socket, dest_ip, dest_port)) {
		// TODO: Raise Error
        *errcode = 1;
		return 0;
	}

	size_t snd_len = eth_udp_write(&ros_socket, buf, len);
	if (snd_len != len) {
		// TODO: Raise Error
        *errcode = 3;
        return snd_len;
	}

	if(!eth_udp_endPacket(&ros_socket)){
		// panic("UDP_XMIT fail");
	}

	return snd_len;
}

size_t transport_eth_read(__unused struct uxrCustomTransport * transport, uint8_t *buf, size_t len, __unused int timeout, uint8_t *errcode)
{
    // make sure the socket is still open
    if (!eth_udp_isopen(&ros_socket)){
		if (!showed_closed){
			puts("UDP Server Closed");
			showed_closed = true;
		}

        *errcode = 2; // TODO: figure out the right error code
		return 0;
	}

    // need to parse the packet first and see if we have data
    size_t packetSize = eth_udp_parsePacket(&ros_socket);
	if (!packetSize){
        *errcode = 2; // TODO: figure out the right error code
		return 0;
	}

    // make sure the packet isnt too big
	if (len < packetSize){
		*errcode = 2; // TODO: figure out the right error code
		return -1;
	}

    // now we can attempt the socket reads
	if (packetSize > 0 && eth_udp_read(&ros_socket, buf, packetSize) != packetSize) {
		*errcode = 2;
		return 0;
	}

    return packetSize;
}

bi_decl(bi_program_feature("Micro-ROS over Ethernet"))

bool transport_eth_init(){
    puts("Initializing W5200");

	// Initialize MAC address with lower values in Flash ID to *randomize* it sort of
	pico_unique_board_id_t uniqueId;
	pico_get_unique_board_id(&uniqueId);
	mac[2] = uniqueId.id[4];
	mac[3] = uniqueId.id[5];
	mac[4] = uniqueId.id[6];
	mac[5] = uniqueId.id[7];

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(ETH_SPI ? spi1 : spi0, 30000000);
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

	//reset routine TODO dont annoy watchdog
    gpio_put(ETH_RST_PIN, 0);
    busy_wait_ms(50);
    gpio_put(ETH_RST_PIN, 1);
    busy_wait_ms(150);		// Max reset time in W5200 datasheet before startup

	if (!eth_init(&eth_device, ETH_SPI ? spi1 : spi0, ETH_CS_PIN, ETH_RST_PIN, mac)){
		puts("Failed to initialize networking!");
		return false;
	}

	// start the Ethernet
  	eth_ifconfig(&eth_device, source_ip, gateway, subnet);

	IPAddress tmp;
	bool mismatch = false;

	w5100_getIPAddress(&eth_device, tmp);
	puts("Configured IP address:");
	for(size_t i = 0; i < 4; i++){
		if(tmp[i] != source_ip[i]){
			printf("IP mismatch after configure byte %d: %d, %d\n", i, tmp[i], source_ip[i]);
			mismatch = true;
		} else {
			printf("%d ", tmp[i]);
		}
	}
	puts("");
	if(mismatch) return false;


	if (!eth_udp_begin(&ros_socket, &eth_device, source_port)) {
		puts("Failed to initialize UDP Server");
		return false;
	}

    rmw_uros_set_custom_transport(
		false,
		NULL,
		transport_eth_open,
		transport_eth_close,
		transport_eth_write,
		transport_eth_read
	);

	return true;
}

bool ethernet_check_online(){
	return w5100_getLinkStatus(&eth_device) == LINK_ON;
}