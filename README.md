# ROS Package for ODrive (3.6 compatible)

This repository contains ROS packages for the [ODrive motor controller](https://odriverobotics.com):

- **`odrive_node`**: Standalone ROS node for communication with ODrives via CAN bus. → [More info](odrive_node/README.md)
- **`odrive_ros_control`**: [work in progress] [ros_control](https://control.ros.org/master/index.html) integration for communication with ODrives via CAN bus.
   → [More info](odrive_ros_control/README.md)
- **`odrive_botwheel_explorer`**: Example for using the `odrive_ros_control` package in the context of the [ODrive BotWheel Explorer](https://shop.odriverobotics.com/products/botwheel-explorer). → [More info](odrive_botwheel_explorer/README.md)

`odrive_node` and `odrive_ros_control` are two alternative approaches and cannot be used at the same time.

For information about installation, prerequisites and getting started, check out the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).

## Compatible Devices

- Makerbase XDrive 3.6

## System Requirements

- Ubuntu >= 20.04
- ROS >= Noetic

## Developer Notes

(For user instructions, see [this guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html) instead.)

You can build this node on a non-ROS developer PC by using the DevContainer configurations in this repository. For example with VSCode:

1. Clone repository
2. Open the repository folder in VSCode. It should automatically present an option "Reopen in Dev Container". Click on that and select the desired ROS version.
3. Once it's re-opened, in the VSCode terminal, run:

   ```bash
   colcon build --packages-select odrive_can
   source ./install/setup.bash
   ```

4. Running the node requires hardware access and only works if the container host is Linux.
