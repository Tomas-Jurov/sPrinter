cmake_minimum_required(VERSION 3.0.2)
project(sprinter_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  GetOrientation.srv
  SetPointArr.srv
  SetPose2D.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)