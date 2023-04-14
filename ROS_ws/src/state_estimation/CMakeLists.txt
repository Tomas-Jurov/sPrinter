cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/state_estimation_node.cpp src/state_estimation.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
