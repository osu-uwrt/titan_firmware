# Project Creation

## Copying a Template

The best way to create a new project is by copying one of the existing template projects in the `examples/` directory.
There are three different templates to pick from

* `pico_sdk_example`: A project implementing the bare minimum to have the project compile with the local pico-sdk
  bundled as part of this repo, enabling only a few additional features such as upload tool and version information.
  This template is only recommended to throw together quick test firmware images to validate part of a board.
* `basic_template`: This project is a basic template which implements the background timer system, titan safety, and
  a core common task that can be executed. This template is useful if you are building firmware which will not use
  MicroROS.
* `micro_ros_template`: This project has all of the necessary boiler plate code to create a MicroROS client, It also
  includes background tasks which can run periodically all the time, or periodically while ROS is running. It also
  has the necessary bindings for titan safety and driving the RGB status LED present on all CAN bus design blocks.
  This template should be used as the base for any project which uses MicroROS.

After selecting your template, copy it and rename it to the corresponding robot's folder. Note if this project isn't
going to be used (and is only for testing), you can create a `scratch_projects` folder in the titan_firmware root
directory and paste the project in there. The scratch projects folder is ignored by git, so you can keep test firmware
without cluttering up the repository.

## Modifying a Template

After copying and renaming the template folder, you need to follow all of the corresponding TODOs present in that
template. The most important is updating the `CMakeLists.txt` to ensure that your project is configured correctly.
You will then need to go through all of the remaining TODOs throughout the c source and header files. These have
all of the important areas that you will need to modify to customize the template for your specific application.

Happy Development!
