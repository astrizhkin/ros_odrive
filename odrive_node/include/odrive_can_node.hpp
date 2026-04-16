#ifndef ODRIVE_CAN_NODE_HPP
#define ODRIVE_CAN_NODE_HPP

#include <ros/ros.h>
//#include <rclcpp/version.h>
#include "odrive_can/ODriveStatus.h"
#include "odrive_can/ControllerStatus.h"
#include "odrive_can/ControlMessage.h"
#include "odrive_can/AxisState.h"
#include "std_srvs/Empty.h"
#include "socket_can.hpp"

#include <mutex>
#include <condition_variable>
#include <array>
#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>

using ODriveStatus = odrive_can::ODriveStatus;
using ControllerStatus = odrive_can::ControllerStatus;
using ControlMessage = odrive_can::ControlMessage;

using AxisState = odrive_can::AxisState;
using Empty = std_srvs::Empty;

class ODriveCanNode {
public:
    ODriveCanNode(const std::string& node_name);
    bool init(EpollEventLoop* event_loop); 
    void deinit();
protected:
    bool send_can_frame_silent(const can_frame& frame) {
        return can_intf_.send_can_frame(frame);
    }
    bool send_can_frame_log(const can_frame& frame, const std::string& message) {
        bool success = can_intf_.send_can_frame(frame);
        if(!success){
            ROS_ERROR("[odrive_can_node] Failed to send CAN frame id=0x%x, %s",frame.can_id,message.c_str());
        }
        return success;
    }
private:
    void recv_callback(const can_frame& frame);
    void subscriber_callback(const ControlMessage::ConstPtr& msg);
    bool service_callback(AxisState::Request& request, AxisState::Response& response);
    bool service_clear_errors_callback(Empty::Request& request, Empty::Response& response);
    void request_state_callback();
    void request_clear_errors_callback();
    void ctrl_msg_callback();
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length);

    ros::NodeHandle nh_;

    uint16_t node_id_;
    bool axis_idle_on_shutdown_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    
    short int ctrl_pub_flag_ = 0;
    std::mutex ctrl_stat_mutex_;
    ControllerStatus ctrl_stat_ = ControllerStatus();
    ros::Publisher ctrl_publisher_;
    
    short int odrv_pub_flag_ = 0;
    std::mutex odrv_stat_mutex_;
    ODriveStatus odrv_stat_ = ODriveStatus();
    ros::Publisher odrv_publisher_;

    EpollEvent sub_evt_;
    std::mutex ctrl_msg_mutex_;
    ControlMessage ctrl_msg_ = ControlMessage();
    //ros::ServiceServer subscriber_;

    EpollEvent srv_evt_;
    uint32_t axis_state_;
    std::mutex axis_state_mutex_;
    std::condition_variable fresh_heartbeat_;
    ros::ServiceServer service_;

    EpollEvent srv_clear_errors_evt_;
    ros::ServiceServer service_clear_errors_;

};

#endif // ODRIVE_CAN_NODE_HPP
