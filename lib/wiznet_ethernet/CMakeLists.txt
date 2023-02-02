
# Setup Wiznet chip name
set(WIZNET_CHIP W5200)
message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")

# Load in ioLibrary_Driver
set(WIZNET_DIR ${CMAKE_CURRENT_LIST_DIR}/ioLibrary_Driver)
include(${CMAKE_CURRENT_LIST_DIR}/ioLibrary_import.cmake)

# Define ioLibrary_Port files
add_library(wiznet_ethernet INTERFACE)

target_sources(wiznet_ethernet INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/ioLibrary_Port/src/w5x00_spi.c
        ${CMAKE_CURRENT_LIST_DIR}/ioLibrary_Port/src/w5x00_gpio_irq.c
        )

target_include_directories(wiznet_ethernet INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/ioLibrary_Port/inc
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(wiznet_ethernet INTERFACE
        wiznet_ethernet_core
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_clocks
        )

# timer
add_library(wiznet_timer INTERFACE)

target_sources(wiznet_timer INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/timer/timer.c
        )

target_include_directories(wiznet_timer INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/timer
        )

target_link_libraries(wiznet_timer INTERFACE
        pico_stdlib
        )
