#include "pico/binary_info.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>

#include "driver/wiznet.h"
#include "titan/binary_info.h"
#include "titan/canmore.h"
#include "titan/debug.h"
#include "titan/logger.h"
#include "titan/safety.h"	// TODO: Only include if saftey support enabled

#include "micro_ros_pico/transport_eth.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "transport_eth"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

typedef uint8_t IPAddress[4];
void ethernet_control_interface_transmit(uint8_t *msg, size_t len);

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Communication Definitions */
// The ROBOT_ variables are defined in the robot header file defined by UWRT_ROBOT
// If these are giving not defined errors, ensure that UWRT_ROBOT is defined in the CMakeLists file for the project
static const uint8_t dest_ip[] = ROBOT_COMPUTER_IP;
static const uint16_t dest_port = ROBOT_COMPUTER_UROS_PORT;
static const uint8_t source_ip[] = ETHERNET_IP;
static const uint16_t source_port = ETHERNET_PORT;
static const IPAddress gateway = ETHERNET_GATEWAY;
static const IPAddress subnet = ETHERNET_MASK;
static uint8_t mac[] = {0x2A, 0xCD, 0xC1, 0x12, 0x34, 0x56};
static udp_socket_t ros_socket, control_interface_socket;
static w5k_data_t eth_device;

static IPAddress heartbeat_broadcast = CANMORE_TITAN_ETH_BROADCAST_IP;
static uint16_t heartbeat_port = CANMORE_TITAN_ETH_HEARTBEAT_BROADCAST_PORT;
static uint16_t control_port = CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT;

// Binary info definitions
bi_decl(bi_program_feature("Micro-ROS over Ethernet"));
bi_decl(bi_device_ip_address_array(source_ip));
bi_decl(bi_agent_ip_address_array(dest_ip));
bi_decl(bi_agent_port(dest_port));

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
        *errcode = 1;
		return 0;
	}

	size_t snd_len = eth_udp_write(&ros_socket, buf, len);
	if (snd_len != len) {
        *errcode = 2;
        return 0;
	}

	if(!eth_udp_endPacket(&ros_socket)){
		*errcode = 3;
		return 0;
	}

	return snd_len;
}

size_t transport_eth_read(__unused struct uxrCustomTransport * transport, uint8_t *buf, size_t len, __unused int timeout, uint8_t *errcode)
{
    // make sure the socket is still open
    if (!eth_udp_isopen(&ros_socket)){
        *errcode = 1;
		return 0;
	}

    // need to parse the packet first and see if we have data
    size_t packetSize = eth_udp_parsePacket(&ros_socket);
	if (!packetSize){
        *errcode = 2;
		return 0;
	}

    // make sure the packet isnt too big
	if (len < packetSize){
		*errcode = 3;
		return 0;
	}

    // now we can attempt the socket reads
	if (packetSize > 0 && eth_udp_read(&ros_socket, buf, packetSize) != packetSize) {
		*errcode = 4;
		return 0;
	}

    return packetSize;
}

bool transport_eth_init(){
	// Initialize MAC address with lower values in Flash ID to *randomize* it sort of
	pico_unique_board_id_t uniqueId;
	pico_get_unique_board_id(&uniqueId);
	mac[2] = uniqueId.id[4];
	mac[3] = uniqueId.id[5];
	mac[4] = uniqueId.id[6];
	mac[5] = uniqueId.id[7];

    // SPI initialization. This use SPI at 30MHz.
	bi_decl_if_func_used(bi_3pins_with_func(ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CLK_PIN, GPIO_FUNC_SPI));
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
	bi_decl_if_func_used(bi_1pin_with_name(ETH_CS_PIN, "W5x00 Chip Select"));
	bi_decl_if_func_used(bi_1pin_with_name(ETH_RST_PIN, "W5x00 Reset"));
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
		LOG_FATAL("Failed to initialize networking!");
		return false;
	}

	// Configure timeouts
	// Note that we don't use TCP, so this really only affects the ARP request
	// Since udp inherently faults, we can drop this down to 1 retry attempt, and cut timeout to 30ms
	// If we don't get a response (within the network mind you, since its ARP), it'll just fail as though the packet
	// transmit failed. And then micro-ROS will either reattempt or just drop the message
	w5100_setRetransmissionTime(&eth_device, 300);	// 30ms timeout
	w5100_setRetransmissionCount(&eth_device, 1);

	// start the Ethernet
  	eth_ifconfig(&eth_device, source_ip, gateway, subnet);

	IPAddress tmp;
	bool mismatch = false;

	w5100_getIPAddress(&eth_device, tmp);
	for(size_t i = 0; i < 4; i++){
		if(tmp[i] != source_ip[i]){
			LOG_FATAL("IP mismatch after configure byte %d: %d, %d\n", i, tmp[i], source_ip[i]);
			mismatch = true;
		}
	}
	if(mismatch) return false;


	if (!eth_udp_begin(&ros_socket, &eth_device, source_port)) {
		LOG_FATAL("Failed to initialize UDP Server");
		return false;
	}

	if (!eth_udp_begin(&control_interface_socket, &eth_device, control_port)) {
		LOG_FATAL("Failed to initialize control interface");
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

	debug_init(&ethernet_control_interface_transmit);

	return true;
}

bool ethernet_check_online(){
	return w5100_getLinkStatus(&eth_device) == LINK_ON;
}

// ========================================
// Heartbeat/Debug Interface
// ========================================

absolute_time_t ethernet_next_heartbeat = {0};
static uint8_t msg_buffer[REG_MAPPED_MAX_REQUEST_SIZE];

void ethernet_control_interface_transmit(uint8_t *msg, size_t len) {
	// This will respond to the last parsed packet
	// So this should respond to the appropriate device
	if (!eth_udp_beginPacket(&control_interface_socket, control_interface_socket.remoteIP, control_interface_socket.remotePort)) {
		return;
	}

	size_t snd_len = eth_udp_write(&control_interface_socket, msg, len);
	if (snd_len != len) {
        return;
	}

	if(!eth_udp_endPacket(&control_interface_socket)){
		return;
	}
}

void ethernet_tick(void) {
    // Heartbeat scheduling
    if (time_reached(ethernet_next_heartbeat)) {
        ethernet_next_heartbeat = make_timeout_time_ms(ETH_HEARTBEAT_INTERVAL_MS);

        static canmore_titan_heartbeat_t heartbeat = {.data = 0};

        heartbeat.pkt.cnt += 1;
        heartbeat.pkt.error = (*fault_list_reg) != 0;
        heartbeat.pkt.mode = CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL;
        heartbeat.pkt.term_enabled = 0;
		heartbeat.pkt.term_valid = 0;

        if (eth_udp_beginPacket(&control_interface_socket, heartbeat_broadcast, heartbeat_port)) {
			if (eth_udp_write(&control_interface_socket, &heartbeat.data, sizeof(heartbeat)) == sizeof(heartbeat)) {
				eth_udp_endPacket(&control_interface_socket);
			}
		}
    }

    // Handle pending requests on control interface
	size_t packetSize;
	do {
		packetSize = eth_udp_parsePacket(&control_interface_socket);
		if (packetSize > 0 && packetSize <= sizeof(msg_buffer)) {
			size_t len = eth_udp_read(&control_interface_socket, msg_buffer, sizeof(msg_buffer));
			if (len == packetSize) {
				debug_process_message(msg_buffer, len);
			}
		}
	} while (packetSize > 0);
}
