#include "odrive_can_node.hpp"
#include "odrive_enums.h"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

enum CmdId : uint32_t {
    kHeartbeat              = 0x001,  // ControllerStatus  - publisher
    kGetError               = 0x003,  // SystemStatus      - publisher
    kSetAxisState           = 0x007,  // SetAxisState      - service
    kGetEncoderEstimates    = 0x009,  // ControllerStatus  - publisher
    kSetControllerMode      = 0x00b,  // ControlMessage    - subscriber
    kSetInputPos,                     // ControlMessage    - subscriber
    kSetInputVel,                     // ControlMessage    - subscriber
    kSetInputTorque,                  // ControlMessage    - subscriber
    kGetIq                  = 0x014,  // ControllerStatus  - publisher
    kGetTemp,                         // SystemStatus      - publisher
    kGetBusVoltageCurrent   = 0x017,  // SystemStatus      - publisher
    kClearErrors            = 0x018,  // ClearErrors       - service
    kGetTorques             = 0x01c,  // ControllerStatus  - publisher
};

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : nh_(node_name) {
    ctrl_publisher_ = nh_.advertise<ControllerStatus>("controller_status", 10);
    odrv_publisher_ = nh_.advertise<ODriveStatus>("odrive_status", 10);
    nh_.subscribe("control_message", 10, &ODriveCanNode::subscriber_callback, this);
    service_              = nh_.advertiseService("request_axis_state", &ODriveCanNode::service_callback, this);
    service_clear_errors_ = nh_.advertiseService("clear_errors", &ODriveCanNode::service_clear_errors_callback, this);
}

void ODriveCanNode::deinit() {
    if (axis_idle_on_shutdown_) {
        struct can_frame frame = {};
        frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
        write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
        frame.can_dlc = 4;
        send_can_frame_log(frame,"AXIS_STATE_IDLE");
    }

    sub_evt_.deinit();
    srv_evt_.deinit();
    can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {
    std::string interface;
    nh_.param<std::string>("interface", interface, "can0");
    nh_.param<int>("node_id", reinterpret_cast<int&>(node_id_), 0);
    nh_.param<bool>("axis_idle_on_shutdown", axis_idle_on_shutdown_, false);

    // Retry CAN interface initialization until it succeeds or ROS shuts down
    ros::Rate retry_rate(3.0); // 1 Hz retry
    while (ros::ok()) {
        if (can_intf_.init(interface, event_loop,
            std::bind(&ODriveCanNode::recv_callback, this, std::placeholders::_1))) {
            break;
        }
        ROS_WARN("Failed to initialize socket CAN interface '%s', retrying in 3s...", interface.c_str());
        retry_rate.sleep();
    }
    
    if (!sub_evt_.init(event_loop, std::bind(&ODriveCanNode::ctrl_msg_callback, this))) {
        ROS_ERROR("Failed to initialize subscriber event");
        return false;
    }
    if (!srv_evt_.init(event_loop, std::bind(&ODriveCanNode::request_state_callback, this))) {
        ROS_ERROR("Failed to initialize service event");
        return false;
    }
    if (!srv_clear_errors_evt_.init(event_loop, std::bind(&ODriveCanNode::request_clear_errors_callback, this))) {
        ROS_ERROR("Failed to initialize clear errors service event");
        return false;
    }

    ROS_INFO("node_id: %d", node_id_);
    ROS_INFO("interface: %s", interface.c_str());
    return true;
}

void ODriveCanNode::recv_callback(const can_frame& frame) {
    if (((frame.can_id >> 5) & 0x3F) != node_id_) return;

    switch (frame.can_id & 0x1F) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.active_errors         = read_le<uint32_t>(frame.data + 0);
            ctrl_stat_.axis_state            = read_le<uint8_t>(frame.data + 4);
            ctrl_stat_.procedure_result      = read_le<uint8_t>(frame.data + 5);
            ctrl_stat_.trajectory_done_flag  = read_le<bool>(frame.data + 6);
            ctrl_pub_flag_ |= 0b0001;
            fresh_heartbeat_.notify_one();
            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000;
            break;
        }
        case CmdId::kSetAxisState:
        case CmdId::kSetControllerMode:
        case CmdId::kSetInputPos:
        case CmdId::kSetInputVel:
        case CmdId::kSetInputTorque:
        case CmdId::kClearErrors: {
            break; // Ignore commands coming from another master/host on the bus
        }
        default: {
            ROS_WARN("Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }

    if (ctrl_pub_flag_ == 0b1111) {
        ctrl_publisher_.publish(ctrl_stat_);
        ctrl_pub_flag_ = 0;
    }
    
    if (odrv_pub_flag_ == 0b111) {
        odrv_publisher_.publish(odrv_stat_);
        odrv_pub_flag_ = 0;
    }
}

void ODriveCanNode::subscriber_callback(const ControlMessage::ConstPtr& msg) {
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg_ = *msg;
    sub_evt_.set();
}

bool ODriveCanNode::service_callback(AxisState::Request& request, AxisState::Response& response) {
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state_ = request.axis_requested_state;
        ROS_INFO("requesting axis state: %d", axis_state_);
    }
    srv_evt_.set();

    // Wait for at least 1 second for a new heartbeat to arrive.
    // If the requested state is something other than CLOSED_LOOP_CONTROL, also
    // wait for the procedure to complete (procedure_result != BUSY).
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    auto call_time = std::chrono::steady_clock::now();
    fresh_heartbeat_.wait(guard, [this, &call_time, &request]() {
        bool is_busy              = this->ctrl_stat_.procedure_result == ODrive6ProcedureResult::PROCEDURE_RESULT_BUSY;
        bool requested_closed_loop = request.axis_requested_state == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
        bool minimum_time_passed  = (std::chrono::steady_clock::now() - call_time >= std::chrono::seconds(1));
        return (requested_closed_loop || !is_busy) && minimum_time_passed;
    });

    response.axis_state      = ctrl_stat_.axis_state;
    response.active_errors   = ctrl_stat_.active_errors;
    response.procedure_result = ctrl_stat_.procedure_result;
    return true;
}

bool ODriveCanNode::service_clear_errors_callback(Empty::Request& /*request*/, Empty::Response& /*response*/) {
    ROS_INFO("clearing errors");
    srv_clear_errors_evt_.set();
    return true;
}

void ODriveCanNode::request_state_callback() {
    uint32_t axis_state;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state = axis_state_;
    }

    struct can_frame frame;

    if (axis_state != 0) {
        // Clear errors if requested state is not IDLE
        frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
        write_le<uint8_t>(0, frame.data);
        frame.can_dlc = 1;
        send_can_frame_log(frame,"CLEAR_ERRORS");
    }

    // Set state
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    write_le<uint32_t>(axis_state, frame.data);
    frame.can_dlc = 4;
    send_can_frame_log(frame,"AXIS_STATE");
}

void ODriveCanNode::request_clear_errors_callback() {
    struct can_frame frame = {};
    frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
    write_le<uint8_t>(0, frame.data);
    frame.can_dlc = 1;
    send_can_frame_log(frame,"CLEAR_ERRORS");
}

void ODriveCanNode::ctrl_msg_callback() {

    uint32_t control_mode;
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
        write_le<uint32_t>(ctrl_msg_.input_mode,   frame.data + 4);
        control_mode = ctrl_msg_.control_mode;
    }
    frame.can_dlc = 8;
    send_can_frame_log(frame,"CONTROLLER_MODE");

    frame = can_frame{};
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            ROS_ERROR("Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            ROS_DEBUG("input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            ROS_DEBUG("input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel,    frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            ROS_DEBUG("input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos,  frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)),    frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }
        default:
            ROS_ERROR("unsupported control_mode: %d", control_mode);
            return;
    }

    send_can_frame_log(frame,"INPUT_CMD");
}

inline bool ODriveCanNode::verify_length(const std::string& name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    ROS_DEBUG("received %s", name.c_str());
    if (!valid) ROS_WARN("Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}
