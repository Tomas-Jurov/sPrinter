cmake_minimum_required(VERSION 3.0.2)
project(gps)

find_package(catkin REQUIRED COMPONENTS
  nmea_navsat_driver
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
