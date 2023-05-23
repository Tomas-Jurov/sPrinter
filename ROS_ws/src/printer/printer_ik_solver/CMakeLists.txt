cmake_minimum_required(VERSION 3.0.2)
project(printer_ik_solver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        tf
        tf2
        tf2_ros
        sprinter_srvs
        tf2_geometry_msgs
        eigen_conversions
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_perception
        pluginlib
        geometric_shapes
        rosbag
)


catkin_package(
        LIBRARIES
        INCLUDE_DIRS
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
        tf2_geometry_msgs
        DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS/sprinter_srvs})
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
        src/printer_ik_solver.cpp src/printer_ik_solver_node.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS
        ${PROJECT_NAME}  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
