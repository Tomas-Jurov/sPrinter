cmake_minimum_required(VERSION 3.0.2)
project(sprinter_gui)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    diagnostic_msgs
    std_srvs
    sprinter_srvs
)

catkin_python_setup()

catkin_package(
)