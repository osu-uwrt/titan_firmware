# Titan Firmware Examples
Example programs for testing setup, starting new projects, or testing out new hardware.

## Example Programs
* `micro_ros_example`: An example Micro-ROS program set up for dual serial transport
* `pico_sdk_example`: A simple hello world and blink program that only uses the pico-sdk

## Creating a Project From Examples
1. Copy an example folder to its destination
    * The project **MUST** be inside of this git repository for it to properly find and configure libraries
2. Open `CMakeLists.txt` and rename the project target name to a name for the new project
    * Note: This is the identifier in the `project()` call
    * It is recommended to use Find-Replace to ensure that all instances have been changed in the file
3. Specify the board/robot by setting `PICO_BOARD` and `UWRT_ROBOT`
4. Rename the project name set in `pico_set_program_name()` to the readable name for the project
5. Configure the pico-sdk for what is needed for the project
    * Configuring the pico-sdk by either linking `micro_ros_usb` or `micro_ros_can`s
    * To enable hardware, such as the adc or pwm, add it to `target_link_libraries`
6. Run a test build
    * Make new build folder and navigate into it: `mkdir build && cd build`
    * Run cmake: `cmake ..`
    * Run make: `make` (The `-j#` flag can be used to make builds faster by specifying the number of threads available)
    * Run `make upload serial` to test on the rp2040 device and pull up a serial monitor
