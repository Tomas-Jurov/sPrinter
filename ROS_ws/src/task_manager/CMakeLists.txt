cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  sprinter_srvs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/task_manager_node.cpp src/task_manager.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
