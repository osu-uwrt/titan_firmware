cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(copro_firmware
    main.c
)

pico_enable_stdio_usb(copro_firmware 1)
pico_enable_stdio_uart(copro_firmware 1)

pico_add_extra_outputs(copro_firmware)

target_link_libraries(copro_firmware pico_stdlib)
