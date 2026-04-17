#include "can_helpers.hpp"
#include "can_simple_messages_5.hpp"
#include "odrive_enums.h"
#include "odrive_can/ODriveStatus.h"
#include "odrive_can/ControllerStatus.h"
#include "pluginlib/class_list_macros.h"
#include "odrive_hardware_interface.hpp"

#include <cmath>

using namespace odrive_ros_control;

static constexpr double STATUS_TIMEOUT_SEC = 1.0;
static constexpr double HEARTBEAT_TIMEOUT_SEC  = 2.0;

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
        ROS_ERROR("[odrive_hi] 'joints' parameter not found");
        return false;
    }

    // Initialize axes from joint list
    for (const auto& joint_name : joint_names) {
        int node_id = 0;
        if (!robot_hw_nh.getParam("joint_node_ids/" + joint_name, node_id)) {
            ROS_ERROR("[odrive_hi] node_id not found for joint '%s'", joint_name.c_str());
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

    // Status publishers
    odrive_status_pub_     = robot_hw_nh.advertise<odrive_can::ODriveStatus>("odrive_status", 10);
    controller_status_pub_ = robot_hw_nh.advertise<odrive_can::ControllerStatus>("controller_status", 10);

    // Initialize CAN interface
    if (!can_intf_.init(can_intf_name_, &event_loop_,
        std::bind(&ODriveHardwareInterface::on_can_msg, this, std::placeholders::_1))) {
        ROS_ERROR("[odrive_hi] Failed to initialize SocketCAN on %s", can_intf_name_.c_str());
        return false;
    }

    ROS_INFO("[odrive_hi] Initialized SocketCAN on %s", can_intf_name_.c_str());

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

    for (auto& axis : axes_) {
        //init time for first timout
        if(axis.odrv_sent_status_stamp_==ros::Time::ZERO) {
            axis.odrv_sent_status_stamp_ = time;
        }
        if(axis.ctrl_sent_status_stamp_==ros::Time::ZERO) {
            axis.ctrl_sent_status_stamp_ = time;
        }

        // --- ODriveStatus ---
        bool odrv_complete = (axis.odrv_pub_flag_ == 0b111);

        bool connected_now = (time - axis.last_heartbeat_stamp_).toSec() < HEARTBEAT_TIMEOUT_SEC;
        if (!connected_now) {
            axis.connected = false;
            ROS_WARN_THROTTLE(5.0, "[odrive_hi] '%s': no heartbeat for %.1fs",
                axis.joint_name_.c_str(),
                (time - axis.last_heartbeat_stamp_).toSec());
        }else{
            bool prev_connected_state = axis.connected;
            axis.connected = true;
            if(!prev_connected_state){
                ROS_INFO("[odrive_hi] '%s': axis connected",axis.joint_name_.c_str());
                //init stamps with current time so we can collect mesages after connection
                axis.ctrl_sent_status_stamp_ = time;
                axis.odrv_sent_status_stamp_ = time;
                set_axis_command_mode(axis);
            }
        }
        bool odrv_timeout  = (time - axis.odrv_sent_status_stamp_).toSec() > STATUS_TIMEOUT_SEC;

        if (odrv_complete || odrv_timeout) {
            odrive_can::ODriveStatus msg;
            msg.header.stamp      = (axis.ctrl_pub_flag_ & 0b001) ? 
                                                //has heartbeat
                                                axis.last_heartbeat_stamp_ : 
                                                time;
            msg.header.frame_id   = axis.joint_name_;
            msg.connected         = axis.connected;
            msg.active_errors     = axis.axis_errors_;
            msg.disarm_reason     = axis.disarm_reason_;//do we need it?
            msg.fet_temperature   = axis.fet_temperature_;
            msg.motor_temperature = axis.motor_temperature_;
            msg.bus_voltage       = axis.bus_voltage_;
            msg.bus_current       = axis.bus_current_;

            if (odrv_timeout && !odrv_complete) {
                ROS_WARN("[odrive_hi] '%s': ODriveStatus timeout (missing fields: 0x%x), last received %.1fs ago",
                    axis.joint_name_.c_str(),
                    axis.odrv_pub_flag_,
                    (time - axis.odrv_sent_status_stamp_).toSec());
            }

            odrive_status_pub_.publish(msg);
            axis.odrv_pub_flag_   = 0;
            axis.odrv_sent_status_stamp_ = time;
        }

        // --- ControllerStatus ---
        bool ctrl_complete = (axis.ctrl_pub_flag_ == 0b0111);
        bool ctrl_timeout  = (time - axis.ctrl_sent_status_stamp_).toSec() > STATUS_TIMEOUT_SEC;

        if (ctrl_complete || ctrl_timeout) {
            odrive_can::ControllerStatus msg;
            msg.header.stamp         =  (axis.ctrl_pub_flag_ & 0b0010) ? 
                                            //has encoder estimates
                                            axis.last_encoder_estimates_stamp_ : 
                                            (axis.ctrl_pub_flag_ & 0b0001) ? 
                                                //has heartbeat
                                                axis.last_heartbeat_stamp_ : 
                                                time;
            msg.header.frame_id      = axis.joint_name_;
            msg.connected            = axis.connected;
            msg.active_errors        = axis.axis_errors_;
            msg.axis_state           = axis.axis_state_;
            msg.procedure_result     = axis.procedure_result_;
            msg.trajectory_done_flag = axis.trajectory_done_flag_;
            msg.pos_estimate         = axis.pos_estimate_;
            msg.vel_estimate         = axis.vel_estimate_;
            msg.iq_setpoint          = axis.iq_setpoint_;
            msg.iq_measured          = axis.iq_measured_;
            //we don't receive periodically torque messages
            //msg.torque_target        = axis.torque_target_;
            //msg.torque_estimate      = axis.torque_estimate_;

            if (ctrl_timeout && !ctrl_complete) {
                ROS_WARN("[odrive_hi] '%s': ControllerStatus timeout (missing fields: 0x%x), last received %.1fs ago",
                    axis.joint_name_.c_str(),
                    axis.ctrl_pub_flag_,
                    (time - axis.ctrl_sent_status_stamp_).toSec());
            }

            controller_status_pub_.publish(msg);
            axis.ctrl_pub_flag_    = 0;
            axis.ctrl_sent_status_stamp_ = time;
        }
    }
}

void ODriveHardwareInterface::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    for (auto& axis : axes_) {
        if(!axis.connected){
            continue;
        }
        if (axis.axis_errors_!=AXIS_ERROR_NONE) {
            Get_Motor_Error_msg_t msg;
            if(!axis.send_silent(msg)){
                ROS_ERROR_THROTTLE(1,"[odrive_hi] Failed to request motor error. Node id=%d",axis.node_id_);
            }
        }

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
            ROS_ERROR_THROTTLE(1,"[odrive_hi] Failed to send can cmd message. Node id=%d",axis.node_id_);
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
    bool axis_found = false;
    uint32_t can_id = (frame.can_id >> 5);
    uint8_t can_cmd = frame.can_id & 0x1f;
    for (auto& axis : axes_) {
        if (can_id == axis.node_id_) {
            axis_found = true;
            axis.on_can_msg(timestamp_, frame);
            //ROS_INFO("[odrive_hi] Got can cmd %d for axis %d",can_cmd, can_id);
        }
    }
    if(!axis_found) {
        ROS_WARN("[odrive_hi] Got can message for unknown axis  %d",can_id);
    }
}

void ODriveHardwareInterface::set_axis_command_mode(const Axis& axis) {
    if (!axis.connected) {
        ROS_INFO("[odrive_hi] Skip control mode setup, axis '%s' is not connected. ",axis.joint_name_.c_str());
        return;
    }
    if (!active_) {
        ROS_INFO("[odrive_hi] Interface inactive. Setting '%s' to idle.",axis.joint_name_.c_str());
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
        ROS_INFO("[odrive_hi] Setting '%s' to position control.",axis.joint_name_.c_str());
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    } else if (axis.vel_input_enabled_) {
        ROS_INFO("[odrive_hi] Setting '%s' to velocity control.",axis.joint_name_.c_str());
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        ROS_INFO("[odrive_hi] Setting '%s' to torque control.",axis.joint_name_.c_str());
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        ROS_INFO("[odrive_hi] No control mode. Setting '%s' to idle.",axis.joint_name_.c_str());
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send_log(state_msg,"AXIS_STATE_IDLE");
        return;
    }

    axis.send_log(control_msg,"control_msg");
    axis.send_log(clear_error_msg,"clear_error_msg");
    axis.send_log(state_msg,"state_msg");
}

//messages we can enable
//+axis.config.can.iq_rate_ms
//+axis.config.can.bus_vi_rate_ms
//+axis.config.can.encoder_rate_ms = 10
//+axis.config.can.heartbeat_rate_ms = 100
//axis.config.can.sensorless_rate_ms
//axis.config.can.encoder_count_rate_ms
//axis.config.can.controller_error_rate_ms
//axis.config.can.encoder_error_rate_ms
//+axis.config.can.motor_error_rate_ms
//axis.config.can.sensorless_error_rate_ms

void Axis::on_can_msg(const ros::Time& timestamp, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;
    bool message_too_short = false;
    switch (cmd) {
        case Heartbeat_msg_t::cmd_id: {
            if (frame.can_dlc < Heartbeat_msg_t::msg_length) {
                message_too_short = true;
                break;
            }
            Heartbeat_msg_t msg;
            msg.decode_buf(frame.data);
            axis_errors_        = msg.Axis_Error;
            axis_state_           = msg.Axis_State;
            procedure_result_     = msg.Procedure_Result;
            trajectory_done_flag_ = msg.Trajectory_Done_Flag;
            ctrl_pub_flag_ |= 0b0001;
            odrv_pub_flag_ |= 0b001;
            last_heartbeat_stamp_ = timestamp;
            break;
        }
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
                message_too_short = true;
                break;
            }
            Get_Encoder_Estimates_msg_t msg;
            msg.decode_buf(frame.data);
            pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
            vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            ctrl_pub_flag_ |= 0b0010;
            last_encoder_estimates_stamp_ = timestamp;
            break;
        }
        case Get_Encoder_Count_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Encoder_Count_msg_t::msg_length) {
                message_too_short = true;
                break;
            }
            Get_Encoder_Count_msg_t msg;
            msg.decode_buf(frame.data);
            // = msg.Shadow_Count;
            // = msg.Count_in_CPR;
            //odrv_pub_flag_ |= 0b001;
            break;
        }
        case Get_Iq_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Iq_msg_t::msg_length) {
                message_too_short = true;
                break;
            }
            Get_Iq_msg_t msg;
            msg.decode_buf(frame.data);
            iq_setpoint_ = msg.Iq_Setpoint;
            iq_measured_ = msg.Iq_Measured;
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        // case Get_Torques_msg_t::cmd_id: {
        //     if (frame.can_dlc < Get_Torques_msg_t::msg_length) {
        //         message_too_short = true;
        //         break;
        //     }
        //     Get_Torques_msg_t msg;
        //     msg.decode_buf(frame.data);
        //     torque_target_   = msg.Torque_Target;
        //     torque_estimate_ = msg.Torque_Estimate;
        //     ctrl_pub_flag_ |= 0b1000;
        //     break;
        // }
        case Get_Motor_Error_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Motor_Error_msg_t::msg_length) {
                message_too_short = true;
                break;
            }
            Get_Motor_Error_msg_t msg;
            msg.decode_buf(frame.data);
            motor_errors_ = msg.Active_Errors;
            disarm_reason_ = msg.Disarm_Reason;
            ROS_ERROR("[odrive_hi] Received '%s' motor error 0x%x",joint_name_.c_str(),motor_errors_);
            //odrv_pub_flag_ |= 0b001;
            break;
        }
        case Get_Temperature_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Temperature_msg_t::msg_length) {
                message_too_short = true;
                break;
            }
            Get_Temperature_msg_t msg;
            msg.decode_buf(frame.data);
            fet_temperature_   = msg.FET_Temperature;
            motor_temperature_ = msg.Motor_Temperature;
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case Get_Bus_Voltage_Current_msg_t::cmd_id: {
            if (frame.can_dlc < Get_Bus_Voltage_Current_msg_t::msg_length) {
                message_too_short = true;
                break;
            }
            Get_Bus_Voltage_Current_msg_t msg;
            msg.decode_buf(frame.data);
            bus_voltage_ = msg.Bus_Voltage;
            bus_current_ = msg.Bus_Current;
            odrv_pub_flag_ |= 0b100;
            break;
        }
        default:
            ROS_WARN("[odrive_hi] Got unknown message axis %d, cmd 0x%x", node_id_, cmd);
            break;

    }
    if(message_too_short) {
        ROS_WARN("[odrive_hi] message 0x%x too short", cmd);
    }

}

PLUGINLIB_EXPORT_CLASS(
    odrive_ros_control::ODriveHardwareInterface,
    hardware_interface::RobotHW
)
