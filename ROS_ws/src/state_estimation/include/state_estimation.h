#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class StateEstimation
{
public:
    StateEstimation(const ros::Publisher &odom_pub)
    : odom_pub_(odom_pub)
    {
    }
    ~StateEstimation() = default;

    void Do();
    
    // Callbacks
    void encodersLeftCallback(const std_msgs::Int8 &msg);
    void encodersRightCallback(const std_msgs::Int8 &msg);
    void locationCallback(const geometry_msgs::Pose2D &msg);
    void gpsCallback(const sensor_msgs::NavSatFix &msg);
    void imuCallback(const sensor_msgs::Imu &msg);
    void tiltCmdCallback(const std_msgs::Int8 &msg);
    void stepper1Callback(const std_msgs::Int32 &msg);
    void stepper2Callback(const std_msgs::Int32 &msg);
    void servo1Callback(const std_msgs::Int16 &msg);
    void servo2Callback(const std_msgs::Int16 &msg);

private:
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
};