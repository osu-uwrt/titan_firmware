#!/bin/sh

set -e

if ! [ -z "$1" ]; then
    if [ "$1" = "del" ]; then
        echo "Deleting..."
        sudo ip link del vcan0
        sudo rmmod vcan
    else
        echo "Unknown arg: $1"
    fi
    exit
fi

echo "Creating Interface"
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
# sudo ip link set vcan0 mtu 16  # For normal CAN
sudo ip link set vcan0 mtu 72  # For CAN FD
sudo ip link set up vcan0
