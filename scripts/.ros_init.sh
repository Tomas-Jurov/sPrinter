#!/bin/bash
LOCAL_IP=`hostname -I | xargs`

export ROS_MASTER_URI=http://192.168.1.57:11311
export ROS_HOSTNAME=$LOCAL_IP
export ROS_IP=$LOCAL_IP
 
echo "ROS_HOSTNAME: "$ROS_HOSTNAME
echo "ROS_IP: "$ROS_IP
echo "ROS_MASTER_URI: "$ROS_MASTER_URI

