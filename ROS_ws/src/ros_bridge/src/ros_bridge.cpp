#include "../include/ros_bridge.h"

ROSBridge::ROSBridge(const ros::Publisher& encoders_left_pub, const ros::Publisher& encoders_right_pub,
    			const ros::Publisher& encoders_location_pub, const ros::Publisher& imu_pub,
    			const ros::Publisher& stepper1_current_pub, const ros::Publisher& stepper2_current_pub,
    			const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub, const ros::Publisher& suntracker_fb_pub)
: encoders_left_pub_(encoders_left_pub)
, encoders_right_pub_(encoders_right_pub)
, encoders_location_pub_(encoders_location_pub)
, imu_pub_(imu_pub)
, stepper1_current_pub_(stepper1_current_pub_)
, stepper2_current_pub_(stepper2_current_pub)
, servo1_pub_(servo1_pub)
, servo2_pub_(servo2_pub)
, suntracker_fb_pub_(suntracker_fb_pub)
{
}

void ROSBridge::Do()
{
}

void ROSBridge::wheelsSpeedTargetCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
}

void ROSBridge::tiltSpeedTargetCallback(const std_msgs::Int8::ConstPtr& msg)
{
}

void ROSBridge::stepper1SpeedCallback(const std_msgs::Int16::ConstPtr& msg)
{
}

void ROSBridge::stepper2SpeedCallback(const std_msgs::Int16::ConstPtr& msg)
{
}

void ROSBridge::stepper1TargetCallback(const std_msgs::Int32::ConstPtr& msg)
{
}

void ROSBridge::stepper2TargetCallback(const std_msgs::Int32::ConstPtr& msg)
{
}

void ROSBridge::servo1TargetCallback(const std_msgs::Int16::ConstPtr& msg)
{
}

void ROSBridge::servo2TargetCallback(const std_msgs::Int16::ConstPtr& msg)
{
}

void ROSBridge::suntrackerCmdCallback(const std_msgs::Empty::ConstPtr& msg)
{
}
