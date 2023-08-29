# Deploying Code to the Vehicle

## Tagging the Commit

Before any pool test which needs new firmware, it is recommended you tag the commit to easily track the code that ran
at the pool test. This allows you to easily track changes between pool tests, making it easier to chase down bugs that
appear at a later test.

Before tagging, ensure you have committed all changes:

    # Run this in the root of the git repo
    git add .
    git commit -m "Your message here"
    git push

You can now tag your current commit: (Note to tag a different commit, add a commit hash after the `git tag` command)

    # An example tag name would be pool-test-2023-08-25 or robosub-2023
    # An example description would be "Talos Involvement Fair Pool Test 8/25/2023"

    git tag -a pool-test-YYYY-MM-DD -m "Your Message Here"
    git push origin pool-test-YYYY-MM-DD

## Compiling All Firmware

To simplify building the firmware, a script has been developed which compiles all firmware and saves it in a specific
directory for archival and deployment. This makes it even easier to roll back to the exact firmware that ran at a
specific test if required.

Ensure this step is performed **after** tagging the commit, to ensure that the compiled-in version information reflects
the git tag.

### First Setup

In the same folder that you cloned `titan_firmware/`, create a `firmware_archive` folder. So if the repo is in
`~/Documents/titan_firmware`, you should make a new folder at `~/Documents/firmware_archive`. Symlink the
`tools/archive.sh` from the titan firmware repo to the new firmware archive folder. This should look like

    # You are in the folder containing titan_firmware
    mkdir firmware_archive
    cd firmware_archive
    ln -s ../titan_firmware/tools/archive.sh .

### Using the script

This archive script will build all of the projects for you and export them into the folder passed as an argument.
Note its recommended to use the tag, but will also work with any arbitrary name, such as needing to quickly recompile
and patch firmware at the pool test. (I typically just append `-patch` to the tag for that folder)

    # Run this in the firmware_archive/ directory
    # If you tagged the commit as pool-test-2023-08-25, you would pass that as the argument

    ./archive.sh [your_tag_here]

After the script runs, you should now have a directory which looks like:

    $ ls pool-test-YYYY-MM-DD/
    actuator_mk2/        camera_cage_bb/  full_ota.tar  smart_battery/
    backplane_firmware/  esc_board/       power_board/

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

To make copying of firmware to the computer easier, a `full_ota.tar` file is created. This contains all of the firmware
OTA images for deployment. Upload this into the `firmware_deploy` directory

    $ sftp ros@orin
    sftp> cd firmware_deploy
    sftp> mkdir ota
    sftp> cd ota
    sftp> put full_ota.tar
    sftp> exit

You can now SSH into the robot computer to upload the firmware. Ensure that you reflash *every* microcontroller. For
ESC board, this requires that you reflash both board 0 and board 1, as well as every smart battery housing.

    $ ssh ros@orin
    ros@orin:~$ cd firmware_deploy/ota
    ros@orin:~/firmware_deploy/ota$ tar -xvf full_ota.tar

    # Run this for every microcontroller
    ros@orin:~/firmware_deploy/ota$ ../bin/upload_tool [replace with image name]_ota.uf2


## Creating a GitHub Release

Publishing GitHub releases allows anyone to easily access the firmware from that pool test in the future, without
needing access to your local `firmware_archive/` directory.

To archive the contents of a pool test folder, run the following command:

    # Run inside the pool-test-YYYY-MM-DD directory

    tar -czvf build_archive.tgz $(ls -d */)

**Note this command ignores any files in the top level pool-test folder, so it won't save any of the tar archives
such as `full_ota.tar`.**

It is recommended to have a copy of the libmicroros install you are building with. If a bug appears in MicroROS, having
a known good version to roll back to is useful, as MicroROS will build with the most recent versions of all the ROS
repositories.

To archive MicroROS, run the following command from your `firmware_archive/` folder:

    tar -czvf libmicroros_build.tgz -C ../titan_firmware/lib/micro_ros_pico/ built_packages available_ros2_types libmicroros/

To publish a release:

1. Ensure you are logged in and authorized on our GitHub org
2. Navigate to [Titan Firmware](https://github.com/osu-uwrt/titan_firmware/) on GitHub
3. Click the Releases header on the right hand side
4. Select the "Draft a New Release" button
5. Add a descriptive title (Ex. "Pool Test Firmware - August 25, 2023")
6. Select the tag name you gave to that release
7. Add the `build_archive.tgz` and the `libmicroros_build.tgz` files to the release
8. Publish the release
