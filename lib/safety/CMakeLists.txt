cmake_minimum_required(VERSION 3.12)

add_library(safety INTERFACE)

target_sources(safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/safety.c
    ${CMAKE_CURRENT_LIST_DIR}/safety_helper.S
)

target_link_libraries(safety INTERFACE 
    basic_logging
    pico_stdlib
    hardware_exception
	
    # Hook required functions for safety watchdog reporting
	"-Wl,--wrap=__assert_func,--wrap=uxr_run_session_until_confirm_delivery,--wrap=uxr_ping_agent_session"
)

target_compile_definitions(safety INTERFACE PICO_PANIC_FUNCTION=safety_panic)

target_include_directories(safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
