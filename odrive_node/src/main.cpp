#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include "socket_can.hpp"
#include <thread>

int main(int argc, char* argv[]) {
    ros::init(argc, argv,"odrive_can");
    EpollEventLoop event_loop;
    auto can_node = std::make_shared<ODriveCanNode>("ODriveCanNode");

    if (!can_node->init(&event_loop)) return -1;

    std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });
    ros::spin();
    can_node->deinit();
    ros::shutdown();
    return 0;
}
