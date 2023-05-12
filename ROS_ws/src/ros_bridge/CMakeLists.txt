cmake_minimum_required(VERSION 3.0.2)
project(ros_bridge)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/ros_bridge_node.cpp src/ros_bridge.cpp src/ros_serial.cpp src/sprinter.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
