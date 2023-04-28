cmake_minimum_required(VERSION 3.0.2)
project(pose_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/pose_control_node.cpp src/pose_control.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
