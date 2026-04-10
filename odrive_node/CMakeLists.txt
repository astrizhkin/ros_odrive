cmake_minimum_required(VERSION 3.0.2)
project(odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
)

# Declare custom messages and services
add_message_files(
  FILES
  ControlMessage.msg
  ControllerStatus.msg
  ODriveStatus.msg
)

add_service_files(
  FILES
  AxisState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include ../odrive_base/include
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_runtime
)

include_directories(
  include
  ../odrive_base/include
  ${catkin_INCLUDE_DIRS}
)

# Build the node executable
add_executable(odrive_can_node
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_can_node.cpp
  src/main.cpp
  include/odrive_can_node.hpp
)

target_compile_features(odrive_can_node PRIVATE cxx_std_17)

add_dependencies(odrive_can_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(odrive_can_node
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS odrive_can_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
