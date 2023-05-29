#!/bin/bash
catkin build sprinter_srvs sprinter_description sprinter_robot_model state_estimation gps_processing imu  -DCMAKE_BUILD_TYPE=Release -j2 -l2
catkin build ros_bridge pose_control task_manager sprinter_control -DCMAKE_BUILD_TYPE=Release -j2 -l2
catkin build printer_control -DCMAKE_BUILD_TYPE=Release -j2 -l2
catkin build printer_ik_solver -DCMAKE_BUILD_TYPE=Release -j2 -l2