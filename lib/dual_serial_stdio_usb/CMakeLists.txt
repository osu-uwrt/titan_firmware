if (TARGET tinyusb_device_unmarked)
    add_library(dual_serial_stdio_usb INTERFACE)

    target_include_directories(dual_serial_stdio_usb INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

    target_sources(dual_serial_stdio_usb INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/reset_interface.c
        ${CMAKE_CURRENT_LIST_DIR}/stdio_usb.c
        ${CMAKE_CURRENT_LIST_DIR}/stdio_usb_descriptors.c
    )

    target_include_directories(dual_serial_stdio_usb INTERFACE ${CMAKE_CURRENT_LIST_DIR})

    target_link_libraries(dual_serial_stdio_usb INTERFACE
        tinyusb_device_unmarked
        pico_stdio
        pico_time
        pico_unique_id
        pico_usb_reset_interface_headers
    )
endif()
