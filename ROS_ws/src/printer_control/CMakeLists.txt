cmake_minimum_required(VERSION 3.0.2)
project(printer_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  sprinter_srvs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/printer_control_node.cpp src/printer_control.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
