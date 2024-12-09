#!/usr/bin/env python3

print("This script gives linuxsocketcan commands to reboot the requested CAN ID even if the controller is not broadcasting a heartbeat")
print("Useful when the CAN controller decides to stop transmitting for whatever reason, but its still receiving")
canid = int(input("Enter CAN ID: "))

assert canid < 32

targetid = 0x3E | (canid << 6)

print()
print("Enter Bootloader:")
print("cansend can0 {}#2100000100000001".format(hex(targetid).replace('0x','').rjust(3, '0').upper()))
print()
print("Exit BL to Application:")
print("cansend can0 {}#4100000600000001".format(hex(targetid).replace('0x','').rjust(3, '0').upper()))

