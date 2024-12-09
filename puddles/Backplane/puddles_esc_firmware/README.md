# Puddles ESC Firmware Directory

This contains the firmware files for the [Blue Robotics Basic ESCs](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/)
on Puddles, as well as the original tones. The firmware on the Blue Robotics ESCs was reflashed to allow for bidirectional
DShot, so RPM data can be properly returned. This is done using the [Bluejay ESC Firmware](https://github.com/bird-sanctuary/bluejay/)
which adds bidirectional dshot support. Additionally, three different firmware images were made which have various startup
tones.

## Configuring/Flashing ESCs

The ESCs can be configured using BLHeliSuite (not BLHeliSuite32). This is an older program for Windows, but allows you
to reflash the ESCs, as well as configure various features such as startup power, direction, and beep volume. This program
is also able to configure an Arduino with the necessary firmware to act as a bridge between the ESC and the computer.

The configuration file for BLHeliSuite can be found in the .ini file in this directory. Additionally, BLHeliSuite can
be found in the teams files / Navionics / Mk 2 Stuff / ESCs.

## Building Firmware

Follow the instructions in the Bluejay repository for building the firmware. We are currently using version `v0.18.2`.
After following bluejay instructions, run the following commands (applies the patch and compiles for UWRT firmware):

    git checkout v0.18.2
    git apply .../path/to/
    make LAYOUT=R MCU=H DEADTIME=15 PWM=24

You should now have compiled firmware for the ESCs
