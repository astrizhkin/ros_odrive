#include "can_helpers.hpp"
#include "can_simple_messages.hpp"

#include "pluginlib/class_list_macros.h"
#include "odrive_hardware_interface.hpp"

#include <cmath>

using namespace odrive_ros_control;

ODriveHardwareInterface::ODriveHardwareInterface() : active_(false) {}

ODriveHardwareInterface::~ODriveHardwareInterface() {
    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }
    can_intf_.deinit();
}

bool ODriveHardwareInterface::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh) {
    // Read parameters
    robot_hw_nh.param<std::string>("can", can_intf_name_, "can0");

    std::vector<std::string> joint_names;
    if (!robot_hw_nh.getParam("joints", joint_names)) {
        ROS_ERROR("ODriveHardwareInterface: 'joints' parameter not found");
        return false;
    }

    // Initialize axes from joint list
    for (const auto& joint_name : joint_names) {
        int node_id = 0;
        if (!robot_hw_nh.getParam("joint_node_ids/" + joint_name, node_id)) {
            ROS_ERROR("ODriveHardwareInterface: node_id not found for joint '%s'", joint_name.c_str());
            return false;
        }
        axes_.emplace_back(&can_intf_, static_cast<uint32_t>(node_id), joint_name);
    }

    // Register hardware interfaces
    for (auto& axis : axes_) {
        // State interface (pos, vel, effort)
        hardware_interface::JointStateHandle state_handle(
            axis.joint_name_,
            &axis.pos_estimate_,
            &axis.vel_estimate_,
            &axis.torque_target_
        );
        joint_state_interface_.registerHandle(state_handle);

        // Command interfaces
        hardware_interface::JointHandle pos_handle(
            joint_state_interface_.getHandle(axis.joint_name_),
            &axis.pos_setpoint_
        );
        position_joint_interface_.registerHandle(pos_handle);

        hardware_interface::JointHandle vel_handle(
            joint_state_interface_.getHandle(axis.joint_name_),
            &axis.vel_setpoint_
        );
        velocity_joint_interface_.registerHandle(vel_handle);

        hardware_interface::JointHandle eff_handle(
            joint_state_interface_.getHandle(axis.joint_name_),
            &axis.torque_setpoint_
        );
        effort_joint_interface_.registerHandle(eff_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_);

    // Initialize CAN interface
    if (!can_intf_.init(can_intf_name_, &event_loop_,
        std::bind(&ODriveHardwareInterface::on_can_msg, this, std::placeholders::_1))) {
        ROS_ERROR("ODriveHardwareInterface: Failed to initialize SocketCAN on %s", can_intf_name_.c_str());
        return false;
    }

    ROS_INFO("ODriveHardwareInterface: Initialized SocketCAN on %s", can_intf_name_.c_str());

    active_ = true;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return true;
}

void ODriveHardwareInterface::read(const ros::Time& time, const ros::Duration& /*period*/) {
    timestamp_ = time;
    while (can_intf_.read_nonblocking()) {
        // drain all pending CAN messages
    }
}

void ODriveHardwareInterface::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    for (auto& axis : axes_) {
        bool sent = true;
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos   = axis.pos_setpoint_ / (2 * M_PI);
            msg.Vel_FF      = axis.vel_input_enabled_    ? (axis.vel_setpoint_ / (2 * M_PI)) : 0.0f;
            msg.Torque_FF   = axis.torque_input_enabled_ ? axis.torque_setpoint_              : 0.0f;
            sent = axis.send_silent(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel        = axis.vel_setpoint_ / (2 * M_PI);
            msg.Input_Torque_FF  = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            sent = axis.send_silent(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;
            sent = axis.send_silent(msg);
        }
        if(!sent){
            ROS_ERROR_STREAM_THROTTLE(2,"[odrive_hi] Failed to send can cmd message. Node id=" << axis.node_id_);
        }
        // no control enabled — don't send any setpoint
    }
}

void ODriveHardwareInterface::doSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        bool mode_switch = false;

        // Disable interfaces claimed by stopping controllers
        for (const auto& ctrl : stop_list) {
            for (const auto& resource : ctrl.claimed_resources) {
                for (const auto& joint : resource.resources) {
                    if (joint != axis.joint_name_) continue;
                    if (resource.hardware_interface == "hardware_interface::PositionJointInterface")
                        { axis.pos_input_enabled_    = false; mode_switch = true; }
                    if (resource.hardware_interface == "hardware_interface::VelocityJointInterface")
                        { axis.vel_input_enabled_    = false; mode_switch = true; }
                    if (resource.hardware_interface == "hardware_interface::EffortJointInterface")
                        { axis.torque_input_enabled_ = false; mode_switch = true; }
                }
            }
        }

        // Enable interfaces claimed by starting controllers
        for (const auto& ctrl : start_list) {
            for (const auto& resource : ctrl.claimed_resources) {
                for (const auto& joint : resource.resources) {
                    if (joint != axis.joint_name_) continue;
                    if (resource.hardware_interface == "hardware_interface::PositionJointInterface")
                        { axis.pos_input_enabled_    = true; mode_switch = true; }
                    if (resource.hardware_interface == "hardware_interface::VelocityJointInterface")
                        { axis.vel_input_enabled_    = true; mode_switch = true; }
                    if (resource.hardware_interface == "hardware_interface::EffortJointInterface")
                        { axis.torque_input_enabled_ = true; mode_switch = true; }
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void ODriveHardwareInterface::set_axis_command_mode(const Axis& axis) {
    if (!active_) {
        ROS_INFO("ODriveHardwareInterface: Interface inactive. Setting axis to idle.");
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send_log(idle_msg,"AXIS_STATE_IDLE");
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    Clear_Errors_msg_t        clear_error_msg;
    Set_Axis_State_msg_t      state_msg;

    clear_error_msg.Identify   = 0;
    control_msg.Input_Mode     = INPUT_MODE_PASSTHROUGH;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.pos_input_enabled_) {
        ROS_INFO("ODriveHardwareInterface: Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    } else if (axis.vel_input_enabled_) {
        ROS_INFO("ODriveHardwareInterface: Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        ROS_INFO("ODriveHardwareInterface: Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        ROS_INFO("ODriveHardwareInterface: No control mode. Setting to idle.");
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send_log(state_msg,"AXIS_STATE_IDLE");
        return;
    }

    axis.send_log(control_msg,"control_msg");
    axis.send_log(clear_error_msg,"clear_error_msg");
    axis.send_log(state_msg,"state_msg");
}

void Axis::on_can_msg(const ros::Time& /*timestamp*/, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
                ROS_WARN("ODriveHardwareInterface: message %d too short", cmd);
                break;
            }
            Get_Encoder_Estimates_msg_t msg;
            msg.decode_buf(frame.data);
            pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
            vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            break;
        }
        case Get_Torques_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Torques_msg_t::msg_length) {
                ROS_WARN("ODriveHardwareInterface: message %d too short", cmd);
                break;
            }
            Get_Torques_msg_t msg;
            msg.decode_buf(frame.data);
            torque_target_   = msg.Torque_Target;
            torque_estimate_ = msg.Torque_Estimate;
            break;
        }
        default:
            break; // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(
    odrive_ros_control::ODriveHardwareInterface,
    hardware_interface::RobotHW
)
