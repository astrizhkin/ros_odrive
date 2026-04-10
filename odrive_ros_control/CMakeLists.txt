cmake_minimum_required(VERSION 3.0.2)
project(odrive_ros_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  odrive_can
  pluginlib
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include ../odrive_base/include
  LIBRARIES odrive_ros_control_plugin
  CATKIN_DEPENDS roscpp hardware_interface controller_manager odrive_can pluginlib
)

include_directories(
  include
  ../odrive_base/include
  ${catkin_INCLUDE_DIRS}
)

# Build the plugin shared library
add_library(odrive_ros_control_plugin SHARED
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_hardware_interface.cpp
)

add_executable(odrive_ros_control_node 
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_hardware_interface.cpp
  src/main.cpp 
)

target_compile_features(odrive_ros_control_node PRIVATE cxx_std_17)
target_link_libraries(odrive_ros_control_node
  ${catkin_LIBRARIES}
)
add_dependencies(odrive_ros_control_plugin
  ${catkin_EXPORTED_TARGETS}
  odrive_can_generate_messages_cpp
)
add_dependencies(
  odrive_ros_control_node
  ${catkin_EXPORTED_TARGETS}
  # explicit dependency on odrive_can message generation
  odrive_can_generate_messages_cpp
)

target_compile_features(odrive_ros_control_plugin PRIVATE cxx_std_17)

target_link_libraries(odrive_ros_control_plugin
  ${catkin_LIBRARIES}
)

# Install the plugin description XML so pluginlib can discover it
install(FILES odrive_hardware_interface.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS odrive_ros_control_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS odrive_ros_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

