cmake_minimum_required(VERSION 3.12)

add_library(safety INTERFACE)

target_sources(safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/crash_reporting.c
    ${CMAKE_CURRENT_LIST_DIR}/src/fault.c
    ${CMAKE_CURRENT_LIST_DIR}/src/kill_switch.c
    ${CMAKE_CURRENT_LIST_DIR}/src/safety.c
    ${CMAKE_CURRENT_LIST_DIR}/src/safety_helper.S
)

target_link_libraries(safety INTERFACE
    basic_logger
    pico_stdlib
    hardware_exception

    # Hook required functions for safety watchdog reporting
	"-Wl,--wrap=__assert_func,--wrap=panic,--wrap=uxr_run_session_until_confirm_delivery,--wrap=uxr_ping_agent_session"
)

target_include_directories(safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
