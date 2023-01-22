# Ethernet
add_library(wiznet_ethernet_core INTERFACE)
if(${WIZNET_CHIP} STREQUAL W5100S)
    target_compile_definitions(wiznet_ethernet_core INTERFACE _WIZCHIP_=W5100S)
elseif(${WIZNET_CHIP} STREQUAL W5500)
    target_compile_definitions(wiznet_ethernet_core INTERFACE _WIZCHIP_=W5500)
elseif(${WIZNET_CHIP} STREQUAL W5200)
    target_compile_definitions(wiznet_ethernet_core INTERFACE _WIZCHIP_=W5200)
else()
    message(FATAL_ERROR "WIZNET_CHIP is wrong = ${WIZNET_CHIP}")
endif()

target_sources(wiznet_ethernet_core INTERFACE
        ${WIZNET_DIR}/Ethernet/socket.c
        ${WIZNET_DIR}/Ethernet/wizchip_conf.c
        )

# Chip specific definitions
if(${WIZNET_CHIP} STREQUAL W5100S)
target_include_directories(wiznet_ethernet_core INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5100S
        )

target_link_libraries(wiznet_ethernet_core INTERFACE
        wiznet_W5100S_specific
        )
elseif(${WIZNET_CHIP} STREQUAL W5200)
target_include_directories(wiznet_ethernet_core INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5200
        )

target_link_libraries(wiznet_ethernet_core INTERFACE
        wiznet_W5200_specific
        )
elseif(${WIZNET_CHIP} STREQUAL W5500)
target_include_directories(wiznet_ethernet_core INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5500
        )

target_link_libraries(wiznet_ethernet_core INTERFACE
        wiznet_w5500_specific
        )
endif()

if(${WIZNET_CHIP} STREQUAL W5100S)
add_library(wiznet_W5100S_specific INTERFACE)

target_sources(wiznet_W5100S_specific INTERFACE
        ${WIZNET_DIR}/Ethernet/W5100S/w5100s.c
        )

target_include_directories(wiznet_W5100S_specific INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5100S
        )

target_link_libraries(wiznet_W5100S_specific INTERFACE
        wiznet_ethernet_core
        )
elseif(${WIZNET_CHIP} STREQUAL W5200)
add_library(wiznet_W5200_specific INTERFACE)

target_sources(wiznet_W5200_specific INTERFACE
        ${WIZNET_DIR}/Ethernet/W5200/w5200.c
        )

target_include_directories(wiznet_W5200_specific INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5200
        )

target_link_libraries(wiznet_W5200_specific INTERFACE
        wiznet_ethernet_core
        )
elseif(${WIZNET_CHIP} STREQUAL W5500)
add_library(wiznet_w5500_specific INTERFACE)

target_sources(wiznet_w5500_specific INTERFACE
        ${WIZNET_DIR}/Ethernet/W5500/w5500.c
        )

target_include_directories(wiznet_w5500_specific INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5500
        )

target_link_libraries(wiznet_w5500_specific INTERFACE
        wiznet_ethernet_core
        )
endif()

# Loopback
add_library(wiznet_loopback INTERFACE)

target_sources(wiznet_loopback INTERFACE
        ${WIZNET_DIR}/Application/loopback/loopback.c
        )

target_include_directories(wiznet_loopback INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Application/loopback
        )

# DHCP
add_library(wiznet_dhcp INTERFACE)

target_sources(wiznet_dhcp INTERFACE
        ${WIZNET_DIR}/Internet/DHCP/dhcp.c
        )

target_include_directories(wiznet_dhcp INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/DHCP
        )

# DNS
add_library(wiznet_dns INTERFACE)

target_sources(wiznet_dns INTERFACE
        ${WIZNET_DIR}/Internet/DNS/dns.c
        )

target_include_directories(wiznet_dns INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/DNS
        )

# FTP Client
add_library(wiznet_ftpclient INTERFACE)

target_sources(wiznet_ftpclient INTERFACE
        ${WIZNET_DIR}/Internet/FTPClient/ftpc.c
        )

target_include_directories(wiznet_ftpclient INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/FTPClient
        )

# FTP Server
add_library(wiznet_ftpserver INTERFACE)

target_sources(wiznet_ftpserver INTERFACE
        ${WIZNET_DIR}/Internet/FTPServer/ftpd.c
        )

target_include_directories(wiznet_ftpserver INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/FTPServer
        )

# HTTP Server
add_library(wiznet_httpserver INTERFACE)

target_sources(wiznet_httpserver INTERFACE
        ${WIZNET_DIR}/Internet/httpServer/httpParser.c
        ${WIZNET_DIR}/Internet/httpServer/httpServer.c
        ${WIZNET_DIR}/Internet/httpServer/httpUtil.c
        )

target_include_directories(wiznet_httpserver INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/httpServer
        )

# MQTT
add_library(wiznet_mqtt INTERFACE)

target_sources(wiznet_mqtt INTERFACE
        ${WIZNET_DIR}/Internet/MQTT/mqtt_interface.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTFormat.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTPacket.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.c
        )

target_include_directories(wiznet_mqtt INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/MQTT
        ${WIZNET_DIR}/Internet/MQTTPacket/src
        )

# SNTP
add_library(wiznet_sntp INTERFACE)

target_sources(wiznet_sntp INTERFACE
        ${WIZNET_DIR}/Internet/SNTP/sntp.c
        )

target_include_directories(wiznet_sntp INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/SNTP
        )
