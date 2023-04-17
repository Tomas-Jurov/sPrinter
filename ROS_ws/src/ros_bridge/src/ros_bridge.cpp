#include "../include/ros_bridge.h"

ROSBridge::ROSBridge(const ros::Publisher& encoders_left_pub, const ros::Publisher& encoders_right_pub,
    			const ros::Publisher& encoders_location_pub, const ros::Publisher& imu_pub, const ros::Publisher& tilt_pub,
    			const ros::Publisher& stepper1_idle_pub, const ros::Publisher& stepper2_idle_pub,
    			const ros::Publisher& stepper1_current_pub, const ros::Publisher& stepper2_current_pub,
    			const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub)
: encoders_left_pub_(encoders_left_pub)
, encoders_right_pub_(encoders_right_pub)
, encoders_location_pub_(encoders_location_pub)
, imu_pub_(imu_pub)
, tilt_pub_(tilt_pub)
, stepper1_idle_pub_(stepper1_idle_pub)
, stepper2_idle_pub_(stepper2_idle_pub)
, stepper1_current_pub_(stepper1_current_pub_)
, stepper2_current_pub_(stepper2_current_pub)
, servo1_pub_(servo1_pub)
, servo2_pub_(servo2_pub)
{
}

void ROSBridge::Do()
{
}

void ROSBridge::leftSpeedTargetCallback(const std_msgs::Int8& msg)
{
}

void ROSBridge::rightSpeedTargetCallback(const std_msgs::Int8& msg)
{
}

void ROSBridge::tiltSpeedTargetback(const std_msgs::Int8& msg)
{
}

void ROSBridge::stepper1SpeedCallback(const std_msgs::Int16& msg)
{
}

void ROSBridge::stepper2SpeedCallback(const std_msgs::Int16& msg)
{
}

void ROSBridge::stepper1TargetCallback(const std_msgs::Int32& msg)
{
}

void ROSBridge::stepper2TargetCallback(const std_msgs::Int32& msg)
{
}

void ROSBridge::servo1TargetCallback(const std_msgs::Int16& msg)
{
}

void ROSBridge::servo2TargetCallback(const std_msgs::Int16& msg)
{
}

bool ROSBridge::suntrackerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
}
