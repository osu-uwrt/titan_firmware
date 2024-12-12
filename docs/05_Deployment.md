# Deploying Code to the Vehicle

## Tagging the Commit

Before any pool test which needs new firmware, it is recommended you tag the commit to easily track the code that ran
at the pool test. This allows you to easily track changes between pool tests, making it easier to chase down bugs that
appear at a later test. The archive script will walk you through the process for tagging the commit.

Before tagging, ensure you have committed all changes:

    # Run this in the root of the git repo
    git add .
    git commit -m "Your message here"
    git push

## Compiling All Firmware

To simplify building the firmware, a script has been developed which compiles all firmware and saves it in a specific
directory for archival and deployment. This makes it even easier to roll back to the exact firmware that ran at a
specific test if required.

### First Setup

In the same folder that you cloned `titan_firmware/`, create a `firmware_archive` folder. So if the repo is in
`~/Documents/titan_firmware`, you should make a new folder at `~/Documents/firmware_archive`. Symlink the
`tools/archive.sh` from the titan firmware repo to the new firmware archive folder. This should look like

    # You are in the folder containing titan_firmware
    mkdir firmware_archive
    cd firmware_archive
    ln -s ../titan_firmware/tools/archive.sh .

### Using the script

This archive script will build all of the projects for you and export them into the requested folder. If you create
a tag, it will name the folder after that tag. If not, it will prompt you for the name instead.

This script also allows you to deploy firmware directly to the vehicle and flash all of the microcontrollers onboard.
If you do that, you must be on the same network as the orin, have the orin added to your system hosts file (should be
done as part of riptide setup), the ~/firmware_deploy/bin folder should have upload tool inside it, and it is
recommended to have ssh keys exchanged so you don't have to enter the password 10 times.

    # Run this in the firmware_archive/ directory
    # It is recommended that you tag the commit if it is being deployed for the vehicle for a test

    ./archive.sh

    # Answer the questions. For the tag, it should be in the format: pool-test-YYYY-MM-DD

After the script runs, you should now have a directory which looks like:

    $ ls pool-test-YYYY-MM-DD/
    actuator_mk2/        camera_cage_bb/  full_ota.tar  smart_battery/
    backplane_firmware/  esc_board/       power_board/  firmware_archive.tgz
    libmicroros_build.tgz

## Deploying to the Vehicle

You can deploy the firmware to the vehicle in two ways:

1. Connecting your computer directly into the CAN Bus network and using Upload Tool as described in Testing Firmware
2. SSHing into the Orin to deploy without opening the vehicle. This is described below

To deploy over SSH, first ensure that the orin is powered up and on the same network as you. Note that this assumes
you already have the hostname set up on your computer and SSH keys established. Follow the software team tutorials
for how to set this up (or just use the IP address and type in the password every time).

### Building Titan Firmware Tools

In the event you don't have a `firmware_deploy` folder (or similar) on the robot computer, or need to update Upload Tool
or Canmore CLI for whatever reason, there is a script to automate deployment.

Copy the tools_build.sh script to the robot computer:

    # Ran in titan_firmware/tools
    $ sftp ros@orin
    sftp> put tools_build.sh
    sftp> exit

You can now ssh into the robot computer and run the script:

    username@your-machine$ ssh ros@orin
    ros@orin:~$ ./tools_build.sh firmware_deploy

The bin directory should now have the following files:

    ros@orin:~$ ls firmware_deploy/bin
    upload_tool*        canmore_cli*

It is now safe to remove the `tools_build.sh` script:

    ros@orin:~$ rm tools_build.sh


### Uploading Firmware on the Robot Computer

If you did not have the archive script deploy the firmware to the boards, you can do so manually.

To make copying of firmware to the computer easier, a `full_ota.tar` file is created. This contains all of the firmware
OTA images for deployment. Upload this into the `firmware_deploy` directory

    $ sftp ros@orin
    sftp> cd firmware_deploy
    sftp> mkdir pool-test-YYYY-MM-DD
    sftp> cd pool-test-YYYY-MM-DD
    sftp> put full_ota.tar
    sftp> exit

You can now SSH into the robot computer to upload the firmware. Ensure that you reflash *every* microcontroller. For
ESC board, this requires that you reflash both board 0 and board 1, as well as every smart battery housing.

    $ ssh ros@orin
    ros@orin:~$ cd firmware_deploy/ota
    ros@orin:~/firmware_deploy/ota$ tar -xvf full_ota.tar

    # Run this for every microcontroller
    ros@orin:~/firmware_deploy/ota$ ../bin/upload_tool [replace with image name]_ota.uf2

### Figuring out what old firmware on orin means

If you ever forget what a random piece of uf2 firmware is, you can identify it using upload tool.

Run `./upload_tool -i my_firmware.uf2` to dump the binary info contained in the file and see when it was built.

## Creating a GitHub Release

Publishing GitHub releases allows anyone to easily access the firmware from that pool test in the future, without
needing access to your local `firmware_archive/` directory.

To publish a release:

1. Ensure you are logged in and authorized on our GitHub org
2. Navigate to [Titan Firmware](https://github.com/osu-uwrt/titan_firmware/) on GitHub
3. Click the Releases header on the right hand side
4. Select the "Draft a New Release" button
5. Add a descriptive title (Ex. "Pool Test Firmware - August 25, 2023")
6. Select the tag name you gave to that release
7. Add the `firmware_archive.tgz` and the `libmicroros_build.tgz` files to the release
8. Publish the release
