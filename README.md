# Titan Firmware
Firmware for OSU UWRT's RP2040 microcontrollers. Written in C using the [Pico SDK](https://github.com/raspberrypi/pico-sdk).

## Features
* **[micro-ROS](https://micro.ros.org/)**: Runs ROS, used by the software stack, directly in firmware to simplify application development. Supports multiple transports:
    * CAN Bus: Used on all Talos MCUs
    * Ethernet: Used on Puddles Backplane
    * USB Serial: Useful for early development
* **Titan Safety**: Core library to ensure all firmware is operating as expected, and give valuable debug information when it is not
    * Kill switch management logic to kill all vehicle actuators on kill signal or loss of heartbeat
    * Fault reporting system to aggregate all firmware or electrical faults into single location
    * Crash reporting features to record reason for last microcontroller reset
    * Profiler to track amount of time spent in each section of code
    * Tied into hardware watchdog timer to enforce safety execution
* **Titan Bootloader**: Allows firmware upgrades over primary vehicle computer without needing to open the electronics system (Supports both Ethernet and CAN bus)
* **CANmore**: Custom CAN bus protocol to allow packets with 128 byte MTU to be sent and reliably re-assembled, with minimal bandwidth overhead
* **CANmore CLI**: Allows in-vehicle debugging over CAN bus link to view in depth diagnostics and issue low-level commands to debug faults
* **Upload Tool**: Unified upload tool supporting USB, SWD, and bootloader interfaces, auto-detection of boards, and version reporting to streamline development and deployment

## Repository Structure

* `docs/`: Documentation for working with Titan Firmware
* `examples/`: Contains example code as a starting point for new projects
* `lib/`: Common code shared across several projects. Libraries are split into driver and titan library types.
* `puddles/`: Code for the puddles vehicle microcontrollers
* `talos/`: Code for the talos vehicle microcontrollers
* `tools/`: Scripts and applications written to run on the host development system to aid in development and deployment. Contains CanmoreCLI and Upload Tool applications.

## Getting Started

### Read the setup documentation [here](docs/01_Setup.md).