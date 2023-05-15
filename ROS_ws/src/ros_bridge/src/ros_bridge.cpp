#include "../include/ros_bridge.h"

ROSbridge::ROSBridge::ROSBridge(const ros::Publisher &wheels_twist_pub
									, const ros::Publisher &stepper1_position_pub
									, const ros::Publisher &stepper2_position_pub
									, const ros::Publisher &servo1_angle_pub
									, const ros::Publisher &servo2_angle_pub
									, const ros::Publisher &suntracker_fb_pub
									, const std::string &port_name
          				, const int baud_rate)
  : wheels_twist_pub_(wheels_twist_pub)
  , stepper1_position_pub_(stepper1_position_pub)
  , stepper2_position_pub_(stepper2_position_pub)
  , servo1_angle_pub_(servo1_angle_pub)
  , servo2_angle_pub_(servo2_angle_pub)
  , suntracker_fb_pub_(suntracker_fb_pub)
	, port_name_(port_name)
	, baud_rate_(baud_rate)
{
  if (!port_name_.empty() && baud_rate_ > 0)
    sprinter_ = std::make_unique<ROSbridge::Sprinter>(port_name_, baud_rate_);
}

void ROSbridge::ROSBridge::setup()
{
  sprinter_->connect();
}

void ROSbridge::ROSBridge::update()
{
  if (sprinter_.get() != nullptr)
  {
    speed_of_wheels_.left_speed = 5;
    speed_of_wheels_.right_speed = -15;
    sprinter_->setSpeedOfWheels(speed_of_wheels_);
    sprinter_->readReturns(&returns_);
    
    wheels_twist_pub_.publish(wheels_twist_msg_);
    servo1_angle_pub_.publish(servo1_angle_msg_);
    servo2_angle_pub_.publish(servo2_angle_msg_);
    stepper1_position_pub_.publish(stepper1_position_msg_);
    stepper2_position_pub_.publish(stepper2_position_msg_);
    suntracker_fb_pub_.publish(suntracker_fb_msg_);
  }
}

void ROSbridge::ROSBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::tiltTargetVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::stepper1SetSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::stepper2SetSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::stepper1TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::stepper2TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::servo1TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::servo2TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
}

void ROSbridge::ROSBridge::suntrackerCmdCallback(const std_msgs::Empty::ConstPtr& msg)
{
}