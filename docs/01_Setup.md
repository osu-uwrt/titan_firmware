# Installation and Setup

**Note:** If you have *riptide_software* on your machine from Software team, you can skip to step 4. This repository
will be located in `~/osu-uwrt/development/titan_firmware`. If you plan to install *riptide_software* at any point on
your machine, run those steps **first** and then return here at step 4, as riptide software installs all necessary
dependencies and clones the repo.

## 1. Linux Setup

This repository requires Linux to build properly. This repository has been tested on Ubuntu 22.04 (or whatever Ubuntu
version Software team is using at this time). If you already have this installed on your machine, you're set.

### WSL Installation

If you are running on Windows, you can use [Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/)
to install Ubuntu in a virtual machine. Check to ensure that you are installing the proper version of Ubuntu - this
should match with the version software team is running. To install WSL, run the following command as **Administrator**:

    wsl --install <Distribution Name>

where the list of distributions can be found using:

    wsl --list --online

Many of the more advanced features of WSL (such as Docker and udev) work better when systemd is enabled. To configure
WSL to use systemd, edit `/etc/wsl.conf` and add the following lines:

    [boot]
    systemd=true

CMake searches for packages and programs using system path, which by default in WSL includes Windows paths. This,
however, *significantly* slows CMake configure and build times, due to the overhead of accessing Windows files from
WSL. This can be disabled by adding the following lines to `/etc/wsl.conf`:

    [interop]
    appendWindowsPath = false

Note that this will disable some of the nicer features of WSL, such as directly calling explorer in your current
directory (instead requiring you to manually navigate there using `\\wsl$`) as well as launching VSCode directly from
your current terminal directory. This step isn't strictly necessary, so if you want these features, you can leave this
config value enabled, at the cost of higher build times.

To apply these changes, run the following in a command prompt window:

    wsl --shutdown


## 2. Installing Required Packages

This repository requires the following external packages:

1. Git
2. Build-Essential (To allow building of various tools on your local machine)
3. CMake (The core build tool used in this repository) and Pkg-Config (to find CMake packages)
4. Arm Cross Compiler
    * gcc-arm-none-eabi
    * libnewlib-arm-none-eabi
    * libstdc++-arm-none-eabi-newlib
5. libusb-1.0-0-dev (LibUSB support for the upload tool)

To install these packages, run the following commands:

    sudo apt update
    sudo apt upgrade
    sudo apt install git build-essential cmake pkg-config gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib libusb-1.0-0-dev

## 3. Set Up the Repository

Navigate to where you would like to clone this repository on your machine. If you are running in WSL, ensure this
location is inside the WSL virtual disk, and not on your operating system drive (beginning with `/mnt/c`), as this
significantly impacts performance.

Clone the repository by running:

    git clone https://github.com/osu-uwrt/titan_firmware
    cd titan_firmware
    git checkout dev

You should now be in the titan firmware repository checked out to the git branch. You now need to clone the submodules.
Both `titan_firmware` and the `pico-sdk` submodule need to have their submodules cloned.

    git submodule update --init
    cd lib/pico-sdk
    git submodule update --init
    cd ../..

The final configuration step on the repository is to ignore changes to your local `.vscode/settings.json`. VSCode
sometimes modifies this file when you do perform certain actions, and to avoid any accidental commits to this file,
this command will keep any local changes to this file ignored:

    git update-index --skip-worktree .vscode/settings.json

Note that if you ever need to pull or commit changes to this file, you should run the command above with
`--no-skip-worktree`. Ensure that you clean out any unnecessary changes to your settings before you commit.

## 4. MicroROS Building

Most of the firmware images in this repository use [Micro-ROS](https://micro.ros.org/) to communicate with the primary
vehicle computer. This must be compiled with support for riptide software-specific messages and other flags for the
RP2040. This is done inside of a docker container provided by Micro-ROS.

To build Micro-ROS, you must first [Install Docker](https://docs.docker.com/engine/install/ubuntu/).

If you are running on Ubuntu, ensure that you restart the machine after adding your account to the docker group after
the install. If you are running in WSL, ensure that you enabled systemd, and run `wsl --shutdown` to restart WSL to
start the docker daemon after the install.

Micro-ROS can be built by running the following script:

    cd lib/micro_ros_pico/
    ./build.sh

After running the build script, a `libmicroros/` directory should now appear in the directory, along with
`libmicroros.a` and an `include/` subfolder.

### Using Someone Else's Build

If you are unable to install docker, it is possible to copy the `libmicroros/` directory from another machine. Copy
the `lib/micro_ros_pico/libmicroros/` folder to your repository, and Micro-ROS firmware should be able to be built.

## 5. Building Firmware

After installing the necessary prerequisites, it is now possible to build firmware images. This repository uses CMake
to manage the build process. To test your install, navigate to any firmware image. In this example, we're going to
build the ESC Board Firmware.

    cd talos/ESCBoard

CMake stores all of its build artifacts in a separate `build/` folder, as this makes it easy to clean by simply
deleting that folder. You must make this folder in in every CMake project you build, as it is ignored by git.

    mkdir build
    cd build

To begin the build process, call CMake to generate the build scripts in your current directory for the project
contained in the parent directory (the directory `..`). Ensure you don't have anything else in the folder where you run
this command, as this will generate a lot of files there.

    cmake ..

Note that the command above doesn't *build* the project, rather it only creates the scripts required to build it. You
now need to call `make` to actually compile the project. Note that `-j8` causes 8 jobs to be spawned, which can run on
several cores. Use the `nproc` command to determine the best number for your machine.

    make -j8

If the build was successful, you should now have one or two UF2 files in your current directory. The UF2 files are the
actual firmware images which get written to the RP2040 microcontroller. To view information about the currently built
application, you can run the following command: (This uses the upload_tool application to extract binary info from the
built application)

    make info

## 6. Setting up VSCode

Visual Studio Code is the preferred editor for this repository. This repo has been preconfigured with several
compilation, formatting, and debug settings in VSCode to ease development.
1. First [download and install](https://code.visualstudio.com/docs/setup/setup-overview) VSCode if you do not already
   have it installed on your machine.
    * Note for WSL users, this should be installed onto Windows, not inside WSL.
2. If you are running on WSL, install the WSL Remote Development Extension (search `ms-vscode-remote.remote-wsl` in the
   extension panel on the left).
3. Open the `titan_firmware` repository in WSL by selecting `File` -> `Open Folder`
    * Note for WSL users, you must first connect to WSL. Press `F1` and type `Connect to WSL`. You can then open a
      folder in the new remote window.
4. Make sure you select `Trust the Authors` to enable all VSCode features.
5. After loading, there will be a dialog box in the bottom right of the screen prompting to install the recommended
   extensions. Click `Install` to ensure that you have all the required extensions for this repository. If you
   accidentally close this dialog, or it does not appear, the extensions pane on the left should contain a category with
   all workspace recommended extensions to install.
6. After the extensions are installed, you will be prompted to select a kit. Be sure to click `[Unspecified]`.
   Unfortunately, you will have to select this every time you change your target project or relaunch VS Code.
7. If prompted to select a release type, select `Debug`. This enables many debug features in the firmware which aid in
   development.
8. If you also have Riptide Software set up on your machine, be sure to disable the ROS extension for this workspace,
   as this adds erroneous include paths to Intellisense.
9. Press the `Build` button on the bottom status bar. If everything was configured properly, you should see
   `Build finished with exit code 0` as the last output.

### CMake Active Project Selection

Your active project controls how IntelliSense performs syntax highlighting, as well as which project is built and
debugged in VSCode.

Note that your active project is listed in the bottom status bar next to a folder icon with a check. This updates
automatically if you start editing a file inside a different project subdirectory. However, if you are editing firmware
libraries, you may need to manually change the active project by clicking the Active Project button on the status bar.
This will pull up a menu of the various projects to choose from. Select a project which includes the library you are
editing, and IntelliSense will start working again.

Every time you change your active project, be sure to press the Build button (in the status bar) to ensure that the
IntelliSense definitions are updated. Sometimes, it may be necessary to force reconfigure the project, such as if you
build the project via command line (as this defaults to a non-debug build). To do so, click the CMake Variant button in
the status bar, and select Debug. This step is necessary, even if it already says Debug, as this action forces CMake to
reconfigure, which does not occur when just pressing Build.

*Try navigating to a file in a project. Ensure that IntelliSense properly updates after running a build.*

**Congratulations! You should now be set up for firmware development!**
