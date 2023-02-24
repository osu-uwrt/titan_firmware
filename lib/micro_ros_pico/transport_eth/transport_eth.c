#include <stdio.h>
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>

#include "safety/safety.h"

#include "micro_ros_pico/transport_eth.h"
#include "eth_networking.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* sock */
#define HTTP_sock_MAX_NUM 4

typedef uint8_t MACAddress[6];
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
static uint8_t source_ip[] = MICRO_IP;
static uint16_t source_port = MICRO_PORT;
static uint8_t mac[] = {0x2A, 0xCD, 0xC1, 0x12, 0x34, 0x56};
static const int sock = MICRO_ROS_PICO_ETH_SOCK_NUM;
static udp_socket_t ros_socket;
static w5k_data_t eth_device;


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
    if (!transport_initialized && ros_socket->socket_active) {
        transport_initialized = true;
    }
    return true;
}

bool transport_eth_close(__unused struct uxrCustomTransport * transport)
{
    //eth_udp_stop(ros_socket);
    return true;
}

size_t transport_eth_write(__unused struct uxrCustomTransport* transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    int snd_len;

    if (!eth_udp_beginPacket(ros_socket, dest_ip, dest_port)) {
		// TODO: Raise Error
        *errcode = 1;
		return 0;
	}

	if (eth_udp_write(ros_socket, buf, len) != len) {
		// TODO: Raise Error
        *errcode = 3;
        return 0;
	}

	eth_udp_endPacket(ros_socket);

	return snd_len;
}

size_t transport_eth_read(__unused struct uxrCustomTransport * transport, uint8_t *buf, size_t len, __unused int timeout, uint8_t *errcode)
{
    if (len > 0 && eth_udp_read(ros_socket, buf, len) != len) {
        *errcode = 2;
		return 0;
	}
    return len;
}

bi_decl(bi_program_feature("Micro-ROS over Ethernet"))

void transport_eth_init(void){
    puts("Initializing W5500");

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(ETH_SPI_HW ? spi1 : spi0, 14*1000*1000);
    spi_set_format( ETH_SPI_HW ? spi1 : spi0,   // SPI instance
                    8,      // Number of bits per transfer
                    0,      // Polarity (CPOL)
                    0,      // Phase (CPHA)
                    SPI_MSB_FIRST);
    gpio_set_function(ETH_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ETH_CLK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(ETH_MOSI_PIN, GPIO_FUNC_SPI);

	seed_random_from_rosc();

    eth_device = eth_init(ETH_SPI_HW ? spi1 : spi0, ETH_CS_PIN, ETH_RST_PIN, mac);

	if (!eth_device){
		puts("Failed to initialize networking!");
		return 0;
	}

	// start the Ethernet
	IPAddress gateway = {192, 168, 1, 1};
	IPAddress subnet = {255, 255, 255, 0};

  	eth_ifconfig(eth_device, source_ip, gateway, subnet);

	ros_socket = eth_udp_begin(eth_device, source_port);
	if (!ros_socket) {
		puts("Failed to initialize UDP Server");
		return 0;
	}

    rmw_uros_set_custom_transport(
		false,
		NULL,
		transport_eth_open,
		transport_eth_close,
		transport_eth_write,
		transport_eth_read
	);
}