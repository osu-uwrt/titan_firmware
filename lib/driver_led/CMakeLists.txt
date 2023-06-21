add_library(driver_led INTERFACE)

target_sources(driver_led INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/led.c
)

target_include_directories(driver_led INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_led INTERFACE
    hardware_clocks
    hardware_gpio
    hardware_pwm
    pico_time
)
