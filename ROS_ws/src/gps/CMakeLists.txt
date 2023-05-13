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

add_custom_command(
  OUTPUT output_file.txt
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gps_install.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Running gps install script..."
)

add_custom_target(run_script ALL DEPENDS output_file.txt)

