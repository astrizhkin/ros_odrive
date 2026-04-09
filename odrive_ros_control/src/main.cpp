#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "odrive_hardware_interface.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "odrive_ros_control");
    ROS_INFO("[odrive_ros_control] Starting");

    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");

    odrive_ros_control::ODriveHardwareInterface odrive;

    if (!odrive.init(nh, robot_hw_nh)) {
        ROS_FATAL("[odrive_ros_control] Failed to initialize hardware interface");
        return 1;
    }

    controller_manager::ControllerManager cm(&odrive, nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0);

    while (ros::ok()) {
        const ros::Time now = ros::Time::now();
        const ros::Duration period = now - prev_time;
        prev_time = now;

        odrive.read(now, period);
        cm.update(now, period);
        odrive.write(now, period);

        rate.sleep();
    }

    spinner.stop();
    ROS_INFO("[odrive_ros_control] Exiting");
    return 0;
}