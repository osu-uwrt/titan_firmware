#!/bin/sh
arm-none-eabi-as -mcpu=cortex-m4 -march=armv6t2 -mthumb dshot_patch.S -o dshot_patch.o
