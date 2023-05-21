cmake_minimum_required(VERSION 3.0.2)
project(sprinter_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  diagnostic_msgs
  std_srvs
  sprinter_srvs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/sprinter_control_node.cpp src/sprinter_control.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
