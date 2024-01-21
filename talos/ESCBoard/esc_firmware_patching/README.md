# APD 80F3 ESC Firmware Patching

As of writing this, the APD 80F3 ESCs do not support 3D dshot mode (read more about dshot [here](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)).
Because of this, if we want to use the fancy dshot features like RPM feedback, we would be stuck to just going in a
single direction (and switching the direciton induces massive time penalty via the swap direction command).
However, it still does support going bidirectional using the old PWM mode, but we would lose all the advantages of
switching to DShot and getting the telemetry. You can see the issue here.

So, I took it upon myself to reverse engineer the firmware running on these ESCs to patch in support for 3D mode. Note
that this patch permanently enables this (the DSHOT_CMD_3D_MODE_OFF doesn't do anything), but for our application that's
fine. It has worked pretty well, there still are some quirks at startup and switching direction (but I think that's more
of a hardware issue and the ESCs struggling to get out of open loop mode because the board design is horrible for EMI
across phases).

## Obtaining the Patched Firmware

Because APD chooses not to release their firmware, I don't feel comfortable putting a copy of it on the public internet.
Instead, you will need to get the fully patched firmware yourself that can be flashed via APD Flashing tool.

There are three ways to get this firmware:
 1. Using the existing patched firmware in the navionics channel in teams, under Mk 2 stuff/ESCs. There should be the
    `APD_F3X_Standard_2.3.0_patched.bin` and `F3x_Boot.bin` files required by APD flashing tool.
 2. There is also an `apd_f3x_2.3.0_3ddshot.patch` file in this directory which can be applied to the `APD_F3X_Standard_2.3.0.bin`
    file in the ESCs folder in Mk 2 stuff. If for whatever reason this folder has been lost, you'll need to obtain a
    copy yourself. That is left as an exercise to the reader.
 3. If you need to recompile or modify the patch assembly, you will need to recreate the patch yourself. I did this using
    Ghidra. There are two sections in the code you need to patch: setting `0x080080b8` to `b 0x0800813c` and setting the
    range `0x0800813c` to `0x0800813f` to a `b.w 0x0800be00` and filling the rest up with `nop`. Then, you insert the
    patched object file in at `0x0800be00` (empty space in flash). This patched object file is made using `./build.sh`,
    and extracting the machine code out of the resulting object file. Note that you can find the Ghidra project in the
    Mk 2 stuff folder which I have already done this, called `APD_F3X_Standard_2.3.0_patched.gzf` (I just copy and paste
    the code section of the .o file into `0x0800be00` and re-export the file as a .bin whenever I need to update the patch)

Additionally, there should be the original ghidra database (`APD_F3X_Standard_2.3.0.gzf`) in the Mk 2 stuff/ESCs folder,
which is the file I used to reverse engineer the firmware and come up with the patch. This is unmodified, and reflects
the original state of the firmware. The patched ghidra database does not have as many symbols inside of it. If you need
to refer to this file, best of luck.

## Flashing the Settings

After reflashing the firmware, you may need to also update the settings. This is using APD Configurator tool (also in
Mk 2 Stuff/ESCs folder). The Talos_Config.cfg should be the most up to date configuration on the ESCs. You can additionally
extract the config from other ESCs to verify.
