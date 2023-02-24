#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "random.h"
#include "eth_networking.h"


typedef uint8_t MACAddress[6];
typedef uint8_t IPAddress[4];


struct packet_data {
	uint32_t address;
	uint16_t port;
	uint16_t packet_id;
};

bool init_networking(MACAddress macAddr, uint8_t* ipAddr) {
	eth_device = eth_init(ETH_SPI_HW ? spi1 : spi0, ETH_CS_PIN, ETH_RST_PIN, macAddr);

	if (!eth_device){
		return false;
	}

	// start the Ethernet
	IPAddress gateway = {192, 168, 1, 1};
	IPAddress subnet = {255, 255, 255, 0};

  	eth_ifconfig(eth_device, ipAddr, gateway, subnet);

	return true;
}

// TODO: Add in link status

bool showed_closed = false;

int getCommand(udp_socket_t server, uint8_t* command_id, uint8_t*data, size_t data_max_size, struct packet_data *packet_data){
	if (!eth_udp_isopen(server)){
		if (!showed_closed){
			puts("UDP Server Closed");
			showed_closed = true;
		}
		// TODO: Raise error
		return -1;
	}

	int packetSize = eth_udp_parsePacket(server);
	if (!packetSize){
		return -1;
	}

	uint8_t* remoteAddr = eth_udp_remoteIP(server);
	uint16_t remotePort = eth_udp_remotePort(server);

	// Read Command Header
	// Format [Packet Length 1-byte] [Command ID 1-byte] [Packet ID 2-bytes] [Data...]
#define COMMAND_HEADER_SIZE 4

	uint8_t header[COMMAND_HEADER_SIZE];
	if (eth_udp_read(server, header, COMMAND_HEADER_SIZE) != COMMAND_HEADER_SIZE){
		// TODO: Raise Error
		return -1;
	}

	if (packetSize != header[0]){
		// TODO: Raise Error
		return -1;
	}

	uint8_t data_length = header[0] - COMMAND_HEADER_SIZE;
	uint8_t _command_id = header[1];
	uint16_t packet_id = (header[2] << 8) | header[3];

	if (data_max_size < data_length){
		// TODO: Raise Error
		return -1;
	}

	if (data_length > 0 && eth_udp_read(server, data, data_length) != data_length) {
		// TODO: Raise Error
		return -1;
	}

	*command_id = _command_id;
	packet_data->packet_id = packet_id;
	packet_data->address = *((uint32_t*)remoteAddr);
	packet_data->port = remotePort;

	return data_length;
}

bool sendResponse(udp_socket_t server, uint8_t *resp_data, size_t resp_data_length, struct packet_data *packet_data){
	if (!eth_udp_isopen(server)){
		// TODO: Raise error
		return false;
	}

#define RESPONSE_COMMAND_HEADER_SIZE 3
	if (resp_data_length > (255 - RESPONSE_COMMAND_HEADER_SIZE)){
		// TODO: Raise error of invalid data size
		return false;
	}

	// Generate response header
	uint8_t response_header[RESPONSE_COMMAND_HEADER_SIZE] = {
		(uint8_t) (resp_data_length + RESPONSE_COMMAND_HEADER_SIZE),	// Total Packet Size
		(uint8_t) (packet_data->packet_id >> 8),						// Upper 8 bits of packet_id
		(uint8_t) (packet_data->packet_id & 0xFF),						// Lower 8 bits of packet_id
	};

	if (!eth_udp_beginPacket(server, (uint8_t*)(&packet_data->address), packet_data->port)) {
		// TODO: Raise Error
		return false;
	}

	if (eth_udp_write(server, response_header, RESPONSE_COMMAND_HEADER_SIZE) != RESPONSE_COMMAND_HEADER_SIZE) {
		// TODO: Raise Error
		return false;
	}

	if (resp_data_length != 0 && eth_udp_write(server, resp_data, resp_data_length) != resp_data_length) {
		// TODO: Raise Error
		return false;
	}

	return eth_udp_endPacket(server);
}


//NOT USED!!
int eth_full_init()
{
    //stdio_init_all();

    puts("Initializing W5500");

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(ETH_SPI_HW ? spi1 : spi0, 14*1000*1000);
    gpio_set_function(ETH_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ETH_CS_PIN,   GPIO_FUNC_SIO);
    gpio_set_function(ETH_CLK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(ETH_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ETH_RST_PIN, GPIO_FUNC_SIO);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(ETH_CS_PIN, GPIO_OUT);
    gpio_put(ETH_CS_PIN, 1);
    
    gpio_set_dir(ETH_RST_PIN, GPIO_OUT);

	//reset routine
    gpio_put(ETH_RST_PIN, 0);
    busy_wait_ms(1500);
    gpio_put(ETH_RST_PIN, 1);
    busy_wait_ms(100);

	seed_random_from_rosc();

	MACAddress mac = {
		0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
	};
	IPAddress ip = {10, 13, 65, 3};

	unsigned int localPort = 2354;

	if (!init_networking(mac, ip)){
		puts("Failed to initialize networking!");
		return 0;
	}

	udp_socket_t server = eth_udp_begin(eth_device, localPort);
	if (!server) {
		puts("Failed to initialize UDP Server");
		return 0;
	}
#if false
    #define DATA_BUFFER_SIZE 253
	struct packet_data packet_data;
	uint8_t* data = (uint8_t*)malloc(DATA_BUFFER_SIZE);
	uint8_t command_id;

	while (!showed_closed) {
		int data_length = getCommand(server, &command_id, data, DATA_BUFFER_SIZE, &packet_data);
		if (data_length < 0){
			busy_wait_ms(1);
			continue;
		}

		if (command_id == 1) {
			uint8_t version_str[2] = {1, 0};
			puts("Version Command Received");
			if (!sendResponse(server, version_str, sizeof(version_str), &packet_data)){
				puts("Failed to send response!");
			}
		} else if (command_id == 3){
			uint8_t status = 1;
			puts("Ping Received");
			if (!sendResponse(server, &status, 1, &packet_data)){
				puts("Failed to send response!");
			}
		} else if (command_id == 4) {
			data[0] = 'F';
			puts("Command 4 received");
			if (!sendResponse(server, data, data_length, &packet_data)){
				puts("Failed to send response!");
			}
		} else if (command_id == 5) {
			eth_udp_stop(server);
			puts("Closing UDP but not cleaning up...");
			break;
		} else if (command_id == 6) {
			eth_deinit(eth_device);
		} else {
			printf("Unknown Command %d Received\n", command_id);
		}
	}

	free(data);
#endif //false

    return 0;
}
