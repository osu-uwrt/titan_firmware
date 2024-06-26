# Testing Firmware

* [Building the Project](#building-the-project)
* [Contents of the Build Directory](#contents-of-the-build-directory)
* [What is Upload Tool](#what-is-upload-tool)
* [USB Interface](#usb-interface)
* [SWD/TagConnect Interface](#swdtagconnect-interface)
* [Bootloader Interface](#bootloader-interface)

## Building the Project

There are two primary ways of building your project:
 * With VSCode
 * With the Command Line

The steps to perform these are explained more in the getting started docs.

Note that unless you are debugging using VSCode, it is eaiser to build on the command line, as you
can then run the `make upload` command to deploy the changes.

When building, there are two build types you'll typically use:
 * `Debug`: Enables all debug checks and extra logging to the firmware. Additionally disables optimizations to make the
    code easier to debug
 * `RelWithDebInfo`: Disables the debug checks, and enables optimizations for normal deployment. However it still
    compiles debug symbols into the ELF so that issues can be easilly debugged. Note that these symbols do not take
    extra space, as they are stripped when creating the UF2 firmware image.

You typically want to run Debug when you are actively troubleshooting, but you should always deploy RelWithDebInfo
builds if it is going to run on the vehicle.

The build type is stored in [CMake Cache](https://cmake.org/cmake/help/book/mastering-cmake/chapter/CMake%20Cache.html)
meaning that you must fully delete the `build/` directory in order to change the build type. For example, if you were
running in RelWithDebInfo (the default when calling `cmake ..`), and wanted to change to Debug, you must run the
following commands:

    rm -r build/
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Debug ..
    make -j8

**Note that VSCode can sometimes change the build type on you, as it shares this `build/` directory. Clean your build
folder whenever switching between VSCode and the terminal.**


## Contents of the Build Directory

After a project successfully compiles, you will be left with several files in the build directory. The most important
files are the `.elf` and `.uf2` files. The ELF file is generated by the toolchain as the full, final, firmware image.
The UF2 file (or files if the bootloader is enabled), are then generated from the ELF file, by tools that extract only
the information that will need to be programmed onto the microcontroller. This makes the file easier to process by other
programs, such as on the RP2040 which needs to be able to decode the raw UF2 file when you upload it with the USB
bootloader.

However, many tools and tasks, such as debugging with GDB, requires the full ELF file. This is because the ELF contains
the symbols and debug information to properly map the raw memory locations in firmware to variable names and file
locations. *It is important to keep the ELF file for any firmware running on the vehicle, so you can debug any issues
if they come up.*

There may be one or two UF2 files, depending on if that project has the bootloader enabled. If you do not
have the bootloader enabled, you will only have one UF2 file. Uploading this file will reflash the entire device with
that firmware image. Enabling the bootloader will generate two UF2 files. One ending in `_ota.uf2` and
another ending in `_with_bl.uf2`. The ota image contains just the compiled project, without the bootloader. This image
is used whenever you are uploading via the bootloader, or just want to upload firmware without modifying the
bootloader. However, the ota image requires a bootloader to be present to work properly. The with bl image will
flash the entire device, including the bootloader. This should be uploaded if you want to fully reflash the device.

**Note that the with bootloader image should not be uploaded with the bootloader unless absolutely necessary, as if
the upload fails or is interrupted, the RP2040 may be left in an unrecoverable state requiring USB or SWD access
(which is difficult for some boards like smart battery housings).**

The `make upload` command uploads the `_with_bl.uf` image, while the `make upload_ota` command uploads the `_ota.uf2`
image. These commands are explained more below.


## What is Upload Tool

Upload Tool is a utility designed to make deploying and inspecting firmware easy. The current version was originally
written to interact with the bootloader, as we have a custom bootloader. But newer versions added support for the
native USB and SWD interfaces, so it can now upload the firmware over every interface on the RP2040. It keeps track of
board serial numbers as well (located in `tools/DeviceList.jsonc`) which will look up serial numbers to board
names/types, so that it is easier to select which device you are uploading to if there are several RP2040s present.

Upload tool is able to:
 * Upload Firmware over the following interfaces:
    * USB Bootrom Mode
    * USB Serial Mode (by rebooting the device into bootloader mode)
    * SWD (via a Picoprobe)
    * CAN Bus (both bootloader and application mode)
    * Ethernet (both bootloader and application mode)
 * Inspect the binary info of a UF2 file, giving project name and build information
 * Catch bootloader during initial boot delay, allowing recovery from broken application images

Upload tool can be located in `tools/upload_tool` and can be built just like any other CMake project. Note that the
firmware projects can link in upload tool, so that it will handle the building and calling of upload tool for you,
instead of requiring you to call it directly. See the sections below for examples. You can also call upload tool
directly, passing it the UF2 firmware file you want to upload.


## USB Interface

### Supported Debug Features
* Firmware Uploading
* Bootloader Uploading
* Debug Serial Console (*Only if USB support is enabled in that project)

### Interface Setup

**For WSL Users:** By default WSL can not access USB devices. Follow Microsoft's guide to
[setting up USBIPD](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) to add USB support.

You need to first ensure that your account has permission to access USB devices. If you have riptide software
installed, this step should have already been performed. If not, run:

    sudo cp tools/rp2040.rules /etc/udev/rules.d/
    sudo adduser $(whoami) plugdev
    sudo udevadm control --reload-rules

Restart your computer (or run `wsl --shutdown` if using WSL) to ensure that the changes apply.

If you want to monitor debug output over the serial port, be sure to install screen by running:

    sudo apt install screen

### Connection Setup

Before connecting via USB, you must first power the board up. There are three possible configurations on UWRT boards:
1. The board takes power directly from USB, **without any protection circuitry**
    * Do **not** connect USB while the board is powered or is _unpowered_ but connected to the vehicle's 5V rail
    * This can damage your computer if there is a voltage difference or your computer attempts to backpower the robot
      from the 5V rail.
    * Ensure the board is unpowered, and all other connectors are removed before attempting to plug in over USB
    * This style was used on older boards, as well as the Smart Battery Housing Microcontroller Board
2. The USB 5V rail is left disconnected on the board
    * The board will need to be powered from an external source to interface with the RP2040
    * This style is common on Mark 2 boards
3. The RP2040 power can switch between primary and USB power
    * USB can be connected while the board is both powered and unpowered
    * This style was used on the 2023 Puddles Backplane Respin

**Ensure you know which configuration applies to your board to before attempting to plug in.**

To interface over USB, connect the USB cable to the computer. RP2040s have two primary USB modes:
* Bootrom Mode
* Application Mode

To enter bootloader mode, hold down the BOOT button on the board while connecting it to power or tapping reset.
The bootrom will see this button being held and enter into a program in bootrom to connect over USB to accept new
firmware. It should appear as a USB mass storage device with the name `RPI-RP2`. *Note that some Mark 2 boards have
damaged BOOT buttons, and may be rather difficult to get into this mode.*

Application mode can be enabled on firmware images by setting `pico_enable_stdio_usb`. This enables a USB serial port
which exposes any logging in firmware. This also exposes an interface to reboot firmware into both the application and
bootrom. Upload Tool uses this so it can reflash the device without needing to have the BOOT button pressed.

### Uploading with Upload Tool

Upload tool can upload in both bootloader mode and normal mode (so long as usb support is compiled in). To upload
firmware with upload tool, run the following command in the project's `build/` directory:

    make upload

If the device was successfully found, you should see a progress bar as it begins to upload firmware.

**Note that there is a bug where normal mode devices struggle to reboot into bootloader mode.** This is due to some
part of the pico-sdk struggling to reboot into bootloader mode when a strict watchdog timer is running. To get
around this, ensure that you are not connected to ROS (so safety is deinitialized) and it is less likely to occur.
You can also manually enter bootrom mode by using the BOOT button, or use a different upload interface.

### Uploading via USB Mass Storage

Firmware can also be uploaded by manually copying the projects's compiled UF2 file to the `RPI-RP2` drive. After the
file is copied, the drive should disconnect and will begin running the new firmware. If the drive does not disappear
after copying, ensure that you are copying a valid UF2 firmware image.

This method works as a good fallback as it does not require any additional software, USB permissions, or (in the case
of WSL or other virtual machines) USB passthrough.

### USB Serial Interface

If the firmware has USB stdio support enabled, it will appear as a USB serial port under `/dev/ttyACMx` where `x` is
some number. To view the list of usb serial ports on your computer, run:

    ls /dev/ttyACM*

If the RP2040 is connected as a USB serial device, you should see at least one entry in this list. To connect to a
device, launch screen with the following arguments (where `ttyACMx` is replaced with a device found above)

    screen /dev/ttyACMx

To exit screen, press `Ctrl+A` (the magic escape sequence), then `k` (to kill), then press 'y' (to confirm).

Note that because USB serial comes up with the device, you will loose any logs which are sent out before you connect
to the serial port. This may cause you to lose early boot logs. If you would like to see these, use the SWD/TagConnect
interface.

## SWD/TagConnect Interface

### Supported Debug Features
* Firmware Uploading
* Bootloader Uploading
* Debug Serial Console
    * Including early boot logging
* GDB Debugging

### Important Terminology

                                                                                +------------Computer-------------+
    +--------+             +--Tag Connect Cable--+     +--------------+         |                     +---------+ |
    |        |  <==SWD==>  |  <======SWD======>  | <=> | Pico running |         | <====CMSIS-DAP====> | OpenOCD | |
    | RP2040 |             |  <=====POWER======  |     |  Picoprobe   | <=USB=> |                     +---------+ |
    |        |  ==UART==>  |  ======UART======>  | ==> |  Firmware    |         | ==USB Serial Port=> | Screen  | |
    +--------+             +---------------------+     +--------------+         |                     +---------+ |
                                                                                +---------------------------------+

* RP2040: ARM Cortex-M0 microcontroller made by Raspberry Pi used on UWRT electronics boards
* Raspberry Pi Pico: A microcontroller development board (like Arduino) which uses the RP2040 microcontroller
* SWD (ARM Serial Wire Debug): Protocol implemented on ARM Cortex microcontrollers which exposes debug access to the CPU
* UART: Communication format which is used on the RP2040 for sending debug serial data
* TagConnect Cable: A cable made by TagConnect which attaches directly to circuit boards for debugging, without the
  need for a connector to be placed on the circuit board.
* Picoprobe: Firmware created by Raspberry Pi to debug RP2040 microcontrollers with another Raspberry Pi Pico. This
  firmware exposes a UART to USB serial converter, as well as a CMSIS-DAP compliant debug probe.
* CMSIS-DAP debug probe: An open protocol specification made by ARM to allow debugging software to easily communicate
  with any probe implementing this specification. Previously, each manufacturer would typically have their own custom
  protocol to communicate with their custom tools.
* OpenOCD: Open source software tool for programming and debugging embedded target devices, such as the RP2040. This
  tool is used by both GDB and Upload Tool to interface with the RP2040 via SWD using the Picoprobe debug adapter.

### Interface Setup

Ensure that you have followed the steps for the [USB Interface Setup](#interface-setup) as they have the same
prerequisites.

You need to first build and install OpenOCD v0.12.0 from source, as Ubuntu 22.04 comes with OpenOCD v0.11.0, which does
not support the RP2040. Ensure you do not have openocd installed from the package manager:

    sudo apt remove --purge --auto-remove openocd

Go to somewhere not tracked by git that you want to store the build files, and run the following commands:

    cd ~/Downloads  # Change this with wherever you want to clone openocd
    git clone --depth 1 --branch v0.12.0 https://github.com/openocd-org/openocd
    cd openocd
    ./bootstrap
    ./configure
    make -j$(nproc)
    sudo make install

You will need to install gdb with ARM support to debug the firmware. This can be installed by running:

    sudo apt install gdb-multiarch

The extension in vscode makes assumptions about which architecture you're debugging based on the gdb executable name.
To make it recognize ARM properly, you will need to run the following command:

    sudo ln -s gdb-multiarch /usr/bin/arm-none-eabi-gdb

### Connection Setup

Before you connect the Picoprobe to the target device, you must switch the jumper depending on if you need to provide
target power via the TagConnect.

When Target Power should be provided by the TagConnect:
* The board being debugged is not powered by any other source
* The board does not have any other power plugs connected
    * This is required as the onboard 3.3V LDO will have reverse current leakage to the 5V rail. If something else is
      connected on the 5V rail, this may pull too much current, **damaging the onboard or Picoprobe's 3.3V regulator.**
    * The boards should not have anything on board which can pull enough current to cause damage, but if board is
      connected to the rest of the vehicle's 5V rail, something may be present which could pull too much current.

Target power is not needed if the board is receiving power from another source.

The sequence to connect is:
1. Select target power
2. Plug the TagConnect cable into the socket on the board under test
3. Connect the picoprobe to your computer's USB port

**If you are using WSL, be sure to follow Microsoft's guide for
[Connecting USB Devices](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

### Uploading Firmware

Firmware can be uploaded by using upload tool with the picoprobe connected by running the following command:

    make upload

If you receive an error on failing to initialize OpenOCD, you can view additional debug information be calling upload
tool with the environment variable `UPLOADTOOL_OPENOCD_EN_STDERR=1`

    cd titan_firmware/tools/upload_tool/build
    UPLOADTOOL_OPENOCD_EN_STDERR=1 ./upload_tool your_uf2_filename_here.uf2

### Accessing Serial Terminal

The picoprobe presents a USB to serial adapter which allows viewing of all debug prints in firmware (so long as stdio
uart is enabled in your project's CMakeLists). The UART TX is sent over the TagConnect cable, allowing for easy access
to this debug terminal. Note that this only supports UART TX, not RX, so screen cannot send data back to the RP2040.

To open this debug terminal, you must first find the serial port for the picoprobe. Run:

    ls /dev/ttyACM*

If you have the picoprobe connected, and only one entry appears, then that is your serial port. If multiple items
appear, you may need to try several items on this list to find the correct one. To connect to a device, launch screen
with the following arguments (where `ttyACMx` is replaced with a device found above). By default, stdio uart is at
115200 baud unless otherwise configured in the project CMakeLists.

    screen /dev/ttyACMx 115200

To exit screen, press `Ctrl+A` (the magic escape sequence), then `k` (to kill), then press 'y' (to confirm).

### Debugging with VSCode

You can debug firmware using the *Run and Debug* menu in VSCode. Ensure that you have the correct active project
selected on the bottom status bar. Then, on Primary Side Bar (on the right), press the Play/Bug icon. You can then press
the play button at the top of the panel for "Pico Debug". You should see the project compile, then another terminal
wlil open for the upload process. If the upload fails with an openocd error, try connecting manually by following the
steps in [Debugging via Command Line](#debugging-via-command-line). If this works, try enabling stderr as explained in
[Uploading Firmware](#uploading-firmware)

After the upload finishes, you should be started at a breakpoint on main. You are now free to debug using VSCode.

### Debugging via Command Line

Before attempting to debug with openocd, ensure that the version running on the target and the .elf file in your build
directory are from the same build. If not, you will run into very weird issues while debugging.

To begin this process, you must start openocd. Open a new terminal and run:

    openocd -f interface/cmsis-dap.cfg -c 'adapter speed 2000' -f target/rp2040.cfg

If the connection succeeds, you should see a message reporting that a GDB server is listening on port 3333. After this
succeeds, you can then open a new tab and connect with GDB. Navigate to the build directory and run:

    # To be ran in the build directory
    gdb-multiarch -ex "target extended-remote :3333" your_filename.elf

You can now debug the firmware as you would any other application with gdb. A useful gdb command is
`monitor reset halt`. This will tell openocd to reset and halt the device, so you can debug from the start of execution.

**Note: You cannot have openocd running and upload over SWD using upload tool, as they both require access to the
picoprobe.**

## Bootloader Interface

### Interface Setup

You do not need to install any tools to upload and interface over CAN bus, although it is recommended to install the
can-utils package to aid in debugging:

    sudo apt install can-utils

Ethernet does not require any additional tools to work.

### Connection Setup

#### Ethernet

The only requirement for the Ethernet interface is that both devices are on the same subnet. This means that if you are
using WSL, you must either configure Hyper-V to use a bridged adapter (quite difficult and probably not worth it), or
pass through a USB to Ethernet adapter to WSL.

Another thing to verify is that you have a valid IP address on the same subnet as the target device. If you connect
through the team router, this should be handled automatically for you. If you connect an Ethernet cable directly into
your computer, you will need to set a static IP address. For desktop ubuntu installs, this must be done via the settings
application.

#### CAN Bus

To connect over CAN bus, you must connect the CAN bus adapter into the bus. This is typically done over the Orin CAN
link on the Camera Cage Breakout Board. Unplug the cable running to the Orin, and connect the USB to CAN bus adapter
to the Camera Cage BB instead. You can then plug the USB to CAN adapter into your computer. For smart battery housings,
this cable is connected into the charge cable.

Ensure that you see a `can0` interface appear in your network interface list by running:

    ip addr

You will now need to set the proper baud rate. Refer to the robot definition header file in
`lib/titan_boards/include/robots` for the most up to date baud rates. As of writing this document, the internal baud
rate is 1 Mbps (1000000) and the external CAN bus is ran at 250 kbps (250000). For example, to configure the adapter
for the internal CAN bus, run:

    ip link set can0 up qlen 1000 type can bitrate 1000000

If a microcontroller is powered up and configured for CAN bus, running this command should display traffic:

    candump can0

### Uploading Firmware

You can run the following command to upload the OTA image over the bootloader:

    make upload_ota

### Uploading in Boot Delay

If you upload firmware which crashes the microcontroller before it is able to receive a firmware update command, it
can be recovered by catching it in the boot delay. Whenever the microcontroller powers up, the LED will flash white
for half a second. This is the bootloader briefly waiting for a request to break into the bootloader, before continuing
boot.

To catch the device in boot delay, run upload tool with the following command, passing in the ota uf2 file:

    # Run in the tools/upload_tool/build directory
    ./upload_tool -w ../path/to/fixed/uf2_ota.uf2

This will ask for the interface, and client ID. For CAN bus devices, this is defined in the board header file. For
Ethernet devices, this is the number after the last dot in the IP address.

You now need to power cycle the microcontroller to allow the bootloader to run. If it is successfully caught in boot
delay, it should begin uploading the firmware.

### Debugging with Canmore CLI

CANmore CLI is a tool which exposes many of the debug features in firmware over the Ethernet/CAN bus interface. This
tool will auto-discover all microcontrollers accessible to the computer, and present a list to select from.

To launch CANmore CLI, navigate to the `tools/canmore_cli` and build as you would any other CMake project. There
should be a `canmore_cli` executable in the folder after the build completes. After selecting a microcontroller, type
`help` to view a list of commands available.

Note that connecting over a TagConnect, exposing GDB and the serial terminal, is often much more powerful than CANmore
CLI. However, this tool is extremely useful when debugging issues found in the water, as this allows crash data and
other telemetry to be pulled from topside. This tool is bundled with the deployment tools (read more in the Deployment
docs), to aid in debugging of issues found in the water. It can also be used to restart misbehaving microcontrollers.

One other use for this tool is printing complex safety structures in an easy-to-view format, such as fault, profiler,
or crash data, which dumping from GDB often requires many conversions by hand.
