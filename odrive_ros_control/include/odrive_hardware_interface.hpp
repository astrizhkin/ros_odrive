#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "odrive_enums.h"
#include "ros/ros.h"
#include "socket_can.hpp"

namespace odrive_ros_control {

class Axis;

class ODriveHardwareInterface : public hardware_interface::RobotHW {
public:
    ODriveHardwareInterface();
    ~ODriveHardwareInterface();

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;
    void doSwitch(
        const std::list<hardware_interface::ControllerInfo>& start_list,
        const std::list<hardware_interface::ControllerInfo>& stop_list
    ) override;

private:
    void on_can_msg(const can_frame& frame);
    void set_axis_command_mode(const Axis& axis);

    bool active_;
    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    ros::Time timestamp_;

    hardware_interface::JointStateInterface    joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface   effort_joint_interface_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id, const std::string& joint_name)
        : can_intf_(can_intf), node_id_(node_id), joint_name_(joint_name) {}

    void on_can_msg(const ros::Time& timestamp, const can_frame& frame);

    SocketCanIntf* can_intf_;
    uint32_t node_id_;
    std::string joint_name_;

    // Commands (ros_control => ODrives)
    double pos_setpoint_    = 0.0; // [rad]
    double vel_setpoint_    = 0.0; // [rad/s]
    double torque_setpoint_ = 0.0; // [Nm]

    // State (ODrives => ros_control)
    double pos_estimate_    = 0.0; // [rad]
    double vel_estimate_    = 0.0; // [rad/s]
    double torque_target_   = 0.0; // [Nm]
    double torque_estimate_ = 0.0; // [Nm]

    // Active control modes
    bool pos_input_enabled_    = false;
    bool vel_input_enabled_    = false;
    bool torque_input_enabled_ = false;

    template <typename T>
    void send(const T& msg) const {
        struct can_frame frame;
        frame.can_id  = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);
        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros_control