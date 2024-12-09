add_library(titan_dual_stdio_usb INTERFACE)

target_include_directories(titan_dual_stdio_usb INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_sources(titan_dual_stdio_usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/reset_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/stdio_usb.c
    ${CMAKE_CURRENT_LIST_DIR}/stdio_usb_descriptors.c
)

target_include_directories(titan_dual_stdio_usb INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(titan_dual_stdio_usb INTERFACE
    tinyusb_device_unmarked
    pico_stdio
    pico_time
    pico_unique_id
    pico_usb_reset_interface_headers
)
