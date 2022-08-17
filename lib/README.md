# Titan Firmware Libraries
Shared library folder for code common to titan_firmware programs
* `dual_serial_stdio_usb`: Contains library to enable a secondary cdc uart port for Micro-ROS while keeping stdio free for debugging
* `micro_ros_pico`: Contains the uart transport definitions for Micro-ROS which will use the secondary cdc port created with `dual_serial_stdio_usb`
* `micro_ros_raspberrypi_pico_sdk` (Submodule): Contains Micro-ROS static library and include files
* `pico-sdk` (Submodule): Bundled pico-sdk for compiling all other programs without needing a separate install
* `uwrt_board`: Include folder for defining RP2040 mappings for UWRT Custom Boards
* `version_tag`: Library to generate version tag data for builds