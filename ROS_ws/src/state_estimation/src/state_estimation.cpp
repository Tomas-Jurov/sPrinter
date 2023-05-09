#include "../include/state_estimation.h"

StateEstimation::StateEstimation(const ros::Publisher& odom_pub) : odom_pub_(odom_pub)
{
}

void StateEstimation::Do()
{
}

void StateEstimation::encodersLeftCallback(const std_msgs::Int8::ConstPtr& msg)
{
}

void StateEstimation::encodersRightCallback(const std_msgs::Int8::ConstPtr& msg)
{
}

void StateEstimation::locationCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
}

void StateEstimation::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
}

void StateEstimation::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
}

void StateEstimation::tiltCmdCallback(const std_msgs::Int8::ConstPtr& msg)
{
}

void StateEstimation::stepper1Callback(const std_msgs::Int32::ConstPtr& msg)
{
}

void StateEstimation::stepper2Callback(const std_msgs::Int32::ConstPtr& msg)
{
}

void StateEstimation::servo1Callback(const std_msgs::Int16::ConstPtr& msg)
{
}

void StateEstimation::servo2Callback(const std_msgs::Int16::ConstPtr& msg)
{
}