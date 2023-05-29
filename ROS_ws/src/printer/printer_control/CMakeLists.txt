cmake_minimum_required(VERSION 3.0.2)
project(printer_control)

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
        diagnostic_msgs
        )

find_package(Eigen3 REQUIRED)

catkin_package(
        LIBRARIES
        INCLUDE_DIRS
        CATKIN_DEPENDS
        tf2_geometry_msgs
        DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
        src/printer_control_node.cpp src/printer_control.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
        ${PROJECT_NAME}  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})