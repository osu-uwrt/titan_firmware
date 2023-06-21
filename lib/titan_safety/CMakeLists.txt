add_library(titan_safety INTERFACE)

target_sources(titan_safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/crash_reporting.c
    ${CMAKE_CURRENT_LIST_DIR}/src/fault.c
    ${CMAKE_CURRENT_LIST_DIR}/src/kill_switch.c
    ${CMAKE_CURRENT_LIST_DIR}/src/safety.c
    ${CMAKE_CURRENT_LIST_DIR}/src/safety_helper.S

    ${CMAKE_CURRENT_LIST_DIR}/src/profiler.c
)

target_link_libraries(titan_safety INTERFACE
    hardware_exception
    hardware_sync
    hardware_watchdog
    pico_time
    titan_logger

    # Hook required functions for safety watchdog reporting
	"-Wl,--wrap=__assert_func,--wrap=panic,--wrap=uxr_run_session_until_confirm_delivery,--wrap=uxr_ping_agent_session,--wrap=watchdog_reboot"
)

target_compile_definitions(titan_safety INTERFACE TITAN_SAFETY=1)

target_include_directories(titan_safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
