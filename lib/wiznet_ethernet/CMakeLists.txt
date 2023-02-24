cmake_minimum_required(VERSION 3.12)

add_library(wiznet_ethernet INTERFACE)

target_sources(wiznet_ethernet INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/base_networking.c
    ${CMAKE_CURRENT_LIST_DIR}/src/udp_networking.c
    ${CMAKE_CURRENT_LIST_DIR}/src/w5100.c
    #${CMAKE_CURRENT_LIST_DIR}/src/w5500_eth.c
)

target_link_libraries(wiznet_ethernet INTERFACE
    pico_stdlib
    hardware_exception

    # Hook required functions for safety watchdog reporting
	"-Wl,--wrap=__assert_func,--wrap=panic,--wrap=uxr_run_session_until_confirm_delivery,--wrap=uxr_ping_agent_session"
)

target_include_directories(wiznet_ethernet INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
