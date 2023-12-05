This contains a UF2 file which is programmed to write the battery pack ID to 0x101FF000.
Hex edit the byte at offset 0x20 in the file to control which ID is written to flash.

This is needed if you manage to fully erase the flash chip on the smart battery, or you need to image a new SBH_MCU PCB.
The ID must match the serial number programmed into the bq40z80 that the MCU board is paired with.
