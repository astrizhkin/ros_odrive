#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "odrive_ros_control/SDO.h"
#include "odrive_enums.h"
#include "ros/ros.h"
#include "socket_can.hpp"
#include <condition_variable>
#include <memory>
#include <mutex>

namespace odrive_ros_control {

struct SDOTransaction {
    enum SdoState : uint8_t { SDO_IDLE, SDO_PENDING, SDO_WAITING, SDO_DONE, SDO_TIMEOUT };

    SdoState sdo_state_ = SDO_IDLE;
    uint8_t  sdo_opcode_ = 0;
    uint16_t sdo_endpoint_id_ = 0;
    uint32_t sdo_request_value_ = 0;
    uint32_t sdo_response_value_ = 0;
    uint8_t  sdo_return_code_ = 0;
    ros::Time sdo_timeout_start_;
    float    sdo_timeout_sec_ = 0.5f;
    std::mutex sdo_mutex_;
    std::condition_variable sdo_cv_;
};

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
    void set_axis_command_mode(Axis& axis);

    bool write_setpoint(const ros::Time& time, Axis& axis);
    bool write_sdo(const ros::Time& time, Axis& axis);

    // Global bus/connection error handling
    canid_t prev_error = 0;
    double last_print_time_sec = 0;
    uint32_t can_error_total_count = 0;
    uint32_t can_error_repeating_count = 0;
    uint32_t cooldown_frames_drop = 0;

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

    // Status publishing
    ros::Publisher odrive_status_pub_;
    ros::Publisher controller_status_pub_;

    // SDO service
    ros::ServiceServer sdo_service_;
    bool sdo_service_callback(odrive_ros_control::SDORequest& req, odrive_ros_control::SDOResponse& res);
};

enum InitState { INIT_IDLE, INIT_DELAY, INIT_SENT };

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id, const std::string& joint_name)
        : can_intf_(can_intf), node_id_(node_id), joint_name_(joint_name) {}

    void on_can_msg(const ros::Time& timestamp, const can_frame& frame);

    SocketCanIntf* can_intf_;
    uint32_t node_id_;
    std::string joint_name_;

    bool connected = false;

    // Reconnection init state machine
    InitState init_state_ = INIT_IDLE;
    ros::Time init_start_time_;
    uint32_t init_retry_count_ = 0;

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

    // ODriveStatus fields
    uint32_t axis_errors_ = 0;
    uint32_t motor_errors_ = 0;
    uint32_t disarm_reason_ = 0;
    float fet_temperature_   = 0.0f;
    float motor_temperature_ = 0.0f;
    float bus_voltage_       = 0.0f;
    float bus_current_       = 0.0f;

    // ControllerStatus fields
    uint8_t axis_state_           = 0;
    uint8_t procedure_result_     = 0;
    bool trajectory_done_flag_    = false;
    float iq_setpoint_            = 0.0f;
    float iq_measured_            = 0.0f;

    // Timestamps of last received CAN messages
    ros::Time last_heartbeat_stamp_ = ros::Time::ZERO;
    ros::Time last_encoder_estimates_stamp_ = ros::Time::ZERO;
    //ros::Time last_iq_stamp_ = ros::Time::ZERO;
    //ros::Time last_torques_stamp_ = ros::Time::ZERO;
    //ros::Time last_error_stamp_ = ros::Time::ZERO;
    //ros::Time last_temperature_stamp_ = ros::Time::ZERO;
    //ros::Time last_bus_voltage_current_stamp_ = ros::Time::ZERO;

    // Timestamps of last send status messages
    ros::Time odrv_sent_status_stamp_ = ros::Time::ZERO;
    ros::Time ctrl_sent_status_stamp_ = ros::Time::ZERO;

    // Publish flags — bitmasks showing which fields have been received
    // ODriveStatus needs: error(001) + temp(010) + bus(100) = 0b111
    // ControllerStatus needs: heartbeat(0001) + encoder(0010) + iq(0100) + torques(1000) = 0b1111
    short int odrv_pub_flag_ = 0;
    short int ctrl_pub_flag_ = 0;

    std::unique_ptr<SDOTransaction> sdo_transaction_ = nullptr;

    template <typename T>
    bool send_silent(const T& msg) {
        struct can_frame frame;
        frame.can_id  = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);
        return can_intf_->send_can_frame(frame);
    }

    template <typename T>
    bool send_log(const T& msg, const std::string& message) {
        uint8_t can_id = node_id_ << 5 | msg.cmd_id;
        bool ok = send_silent(msg);
        if (!ok) {
            ROS_ERROR("[odrive_hi] Failed to send CAN frame id=0x%x, %s", can_id, message.c_str());
        }
        return ok;
    }

};

} // namespace odrive_ros_control
