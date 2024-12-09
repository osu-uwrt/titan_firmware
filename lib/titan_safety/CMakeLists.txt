add_library(titan_safety INTERFACE)

target_sources(titan_safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/crash_reporting.c
    ${CMAKE_CURRENT_LIST_DIR}/src/fault.c
    ${CMAKE_CURRENT_LIST_DIR}/src/kill_switch.c
    ${CMAKE_CURRENT_LIST_DIR}/src/multicore.c
    ${CMAKE_CURRENT_LIST_DIR}/src/safety.c
    ${CMAKE_CURRENT_LIST_DIR}/src/safety_helper.S

    ${CMAKE_CURRENT_LIST_DIR}/src/profiler.c
)

target_link_libraries(titan_safety INTERFACE
    hardware_exception
    hardware_sync
    hardware_watchdog
    pico_multicore
    pico_time
    titan_logger

    # Hook required functions for safety watchdog reporting
    "-Wl,--wrap=__assert_func,--wrap=panic,--wrap=hard_assertion_failure,--wrap=watchdog_reboot"
)

target_compile_definitions(titan_safety INTERFACE TITAN_SAFETY=1)
target_compile_definitions(titan_safety INTERFACE PICO_STDIO_USB_RESET_RESET_TO_FLASH_DELAY_MS=0)
target_compile_definitions(titan_safety INTERFACE PICO_PANIC_FUNCTION=safety_panic)

target_include_directories(titan_safety INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
