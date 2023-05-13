# Define all normal libraries
add_subdirectory(async_i2c/)
add_subdirectory(basic_logger/)
add_subdirectory(basic_queue/)
add_subdirectory(can_bus/)
add_subdirectory(dual_serial_stdio_usb/)
add_subdirectory(micro_ros_pico/)
add_subdirectory(safety/)
add_subdirectory(sht41/)
add_subdirectory(status_strip/)
add_subdirectory(wiznet_ethernet/)
add_subdirectory(version_tag/)
add_subdirectory(dynamixel/)

# TODO: fix when repo is reoganized
add_subdirectory(${REPO_DIR}/bootloader/titan_binary_info titan_binary_info)