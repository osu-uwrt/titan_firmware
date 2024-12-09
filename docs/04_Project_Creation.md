# Creating a New Project

## Copying from a template

Whenever you create a new project, you should copy it from one of the existing templates in the `examples/` directory.
This consists of the following templates:
 * `micro_ros_template`: The template to be used for any new firmware running MicroROS
 * `basic_template`: The template when you are developing firmware which will not be connecting to the robot via
    MicroROS (for example, the smart battery charger)
 * `pico_sdk_example`: A bare minimum project to get code compiling with the pico-sdk. This should not be used for
    production firmware, and should only be used for basic testing.

After selecting a template, copy the folder to the destination folder. If that firmware is running on the robot, copy
it to the robot's corresponding directory. Note, if you are doing testing, you can make a `scratch_projects/` folder in
the `titan_firmware` repo root directory. This folder is ignored by git, so you can keep test firmware in there without
accidentally committing them to git.

## Editing the CMakeLists.txt

Once you have copied the example folder, you must edit the `CMakeLists.txt` file to fill out all of the required TODO
items.

## Adding your Board (optional)

If this is the first piece of firmware supporting this board/robot, you will need to add the corresponding board/robot
definition into the `lib/titan_boards` directory. See other boards for example. The board definition file gives human
readable names to the various GPIO pins on the device, as well as configures default behavior for many of the libraries
inside the pico-sdk as well as titan firmware. This configures things such as if the board has CAN bus, which CAN bus
it is attached to, as well as that board's CAN bus ID, which pins UART is accessible on, what the clock frequency is,
the flash size, and much more. If this file is not made properly, firmware may fail to compile or crash in unexpected
and odd ways.

If you are using the same RP2040 design block that was used in the mark 2 electronics, you can include this common
header file and only define the other pins for your board (see other mark 2 boards as an example). However, if your
board is a brand new RP2040 block design, be sure to define all of the required preprocessor definitions so the SDK
knows how to properly compile the firmware for that target (see the RP2040 design block for an example, as well as
the pico.h included in the pico-sdk builtin board definitions).

The robot definition file is used to define the various networks that the vehicle has on it so that all boards
targetting that vehicle have a shared network configuration. See the robots as an example, for how ethernet/can bus
is configured. As the vehicle electronics design evolves, the requirements for this file with almost certinaly change,
but this file is a great place to keep these common definitions that change from vehicle to vehilcle.

## Adding project to your settings.json

To enable VSCode Intellisense (syntax highlighting, tooltips, etc.) you must add the project to the `settings.json`.
After adding the project to that file, you must either have vscode reload the current window, or restart the vscode
application. Although sometimes the project might appear without a restart, intellisense will act weird until everything
is fully restarted.

Note if your project is in `scratch_projects/`: Make sure to not commit this change, as other people will not be able
to access that project in your scratch_projects. It's always good practice to check over every file you're committing
before doing so, since a lot of times things like this can slip in and break other people's workspaces.

## Building the Project for the First Time

Now that the project has been configured, you should be able to select the project via the CMake configuration panel
and press build. So long as you set up your cmakelists/board configuration correctly, the project should build. If not,
be sure to look at the error and try to figure out if there's any TODOs you missed in the CMakeLists, or if you made
a new board/robot header file, all of the various #defines are present from other boards. Often times these errors
are cryptic, but are related to configuration issues such as setting a CAN Bus Micro ROS Transport on a board that only
has Ethernet.

## Getting Started

You should now be able to open `main.c` and after a intellisense loads, everything should be lit up without errors.
You can now begin going through the various project files, filling in all of your code into the various TODOs throughout
the source code. Some of these should be filled out right away, while others can be filled out as you develop the
project. However, be sure to remove all of the TODOs before you are ready to "deploy" the firmware.

## Adding Complexity

If you need to add complex features that might be used across multiple boards, this is a great candidate to make it into
a library. Things such as an i2c analog to digital converter or an LED controller can have its code moved out of the
core firmware project and into the `lib/` folder. The best way to do this is to find an existing library with a similar
level of complexity to your new component, copy that library to a new folder, and modify it to fit your needs.

## Additional Resources

There are 3 ways to learn more about how to develop your firmware project:
1. Pico SDK exmaples: You can find a lot of guides online on how to write programs with the pico-sdk. Things such as
   controlling PWM, reading the ADC, or controlling GPIO pins can all be copy pasted from these tutorials and used in
   your firmware.
2. Looking at how other firmware projects use that feature: There's a fair amount of firmware already written. Chances
   are some piece of firmware already does part of what you want. Search through this file to see if you can find
   something which does something similar and try to copy that part into your code.
3. Reading the source code: If you can't find any examples online or in other pieces of firmware, try reading through
   the source code/header files. These often times contain the docs in header for how to use the functions, and you can
   read through the source code to see how that function works so you can get your code to do it. When doing this,
   especially if you're working close to the hardware, be sure to read the RP2040 datasheet to get a full understanding
   of how the hardware works, and how the SDK interacts with it.

Happy Developing!
