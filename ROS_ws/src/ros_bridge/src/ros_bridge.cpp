#include "../include/ros_bridge.h"

ROSbridge::ROSBridge::ROSBridge(const ros::Publisher& wheels_twist_pub, const ros::Publisher& stepper1_position_pub,
                                const ros::Publisher& stepper2_position_pub, const ros::Publisher& servo1_angle_pub,
                                const ros::Publisher& servo2_angle_pub, const ros::Publisher& suntracker_fb_pub)
  : wheels_twist_pub_(wheels_twist_pub)
  , stepper1_position_pub_(stepper1_position_pub)
  , stepper2_position_pub_(stepper2_position_pub)
  , servo1_angle_pub_(servo1_angle_pub)
  , servo2_angle_pub_(servo2_angle_pub)
  , suntracker_fb_pub_(suntracker_fb_pub)
{
}

void ROSbridge::ROSBridge::loadParams(const ros::NodeHandle &nh)
{
  getParam(nh, "mcu_serial/port", &params_.port_name);
  getParam(nh, "mcu_serial/baud", &params_.baud_rate);
  getParam(nh, "transmission_params/servo1_gain", &params_.servo1_gain);
  getParam(nh, "transmission_params/servo1_offset", &params_.servo1_offset);
  getParam(nh, "transmission_params/servo2_gain", &params_.servo2_gain);
  getParam(nh, "transmission_params/servo2_offset", &params_.servo2_offset);
  getParam(nh, "transmission_params/stepper1", &params_.stepper1_gain);
  getParam(nh, "transmission_params/stepper2", &params_.stepper2_gain);
}

template<class T> void ROSbridge::ROSBridge::getParam(const ros::NodeHandle &nh, const std::string &name, T* storage) const
{
  if (!nh.getParam(name, *storage))
    ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
}

void ROSbridge::ROSBridge::setup()
{
  if (!params_.port_name.empty() && params_.baud_rate > 0)
    sprinter_ = std::make_unique<ROSbridge::Sprinter>(params_.port_name, params_.baud_rate);
  sprinter_->connect();
}

void ROSbridge::ROSBridge::update()
{
  if (sprinter_.get() != nullptr)
  {
    getAndPublishReturns();
  }
}

void ROSbridge::ROSBridge::getAndPublishReturns()
{
  sprinter_->readReturns(&returns_);

  wheels_twist_msg_.linear.x = (R / 2) * deg2rad<double>(returns_.right_grp_velocity + returns_.left_grp_velocity);
  wheels_twist_msg_.angular.z = (R / B) * deg2rad<double>(returns_.right_grp_velocity - returns_.left_grp_velocity);

  stepper1_position_msg_.data = params_.stepper1_gain * returns_.stepper1_current_steps;
  stepper2_position_msg_.data = params_.stepper2_gain * returns_.stepper2_current_steps;

  servo1_angle_msg_.data = params_.servo1_gain * (returns_.servo1_current_angle-params_.servo1_offset);
  servo2_angle_msg_.data = params_.servo2_gain * (returns_.servo2_current_angle-params_.servo2_offset);

  suntracker_fb_msg_.data = returns_.suntracker_done;

  wheels_twist_pub_.publish(wheels_twist_msg_);

  servo1_angle_pub_.publish(servo1_angle_msg_);
  servo2_angle_pub_.publish(servo2_angle_msg_);

  stepper1_position_pub_.publish(stepper1_position_msg_);
  stepper2_position_pub_.publish(stepper2_position_msg_);

  suntracker_fb_pub_.publish(suntracker_fb_msg_);
}


void ROSbridge::ROSBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  velocity_of_wheels_.left = rad2deg<short>((2 * msg->linear.x / R - B * msg->angular.z / R) / 2);
  velocity_of_wheels_.right = rad2deg<short>((2 * msg->linear.x / R + B * msg->angular.z / R) / 2);
  sprinter_->setVelocityOfWheels(velocity_of_wheels_);
}

void ROSbridge::ROSBridge::tiltTargetVelCallback(const std_msgs::Int8::ConstPtr& msg)
{
  sprinter_->setVelocityOfLinearActuator(msg->data);
}

void ROSbridge::ROSBridge::stepper1SetSpeedCallback(const std_msgs::Int16::ConstPtr& msg)
{
  sprinter_->setSpeedOfStepper1(static_cast<uint16_t>(msg->data));
}

void ROSbridge::ROSBridge::stepper2SetSpeedCallback(const std_msgs::Int16::ConstPtr& msg)
{
  sprinter_->setSpeedOfStepper2(static_cast<uint16_t>(msg->data));
}

void ROSbridge::ROSBridge::stepper1TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
  sprinter_->stepper1SetTargetSteps(static_cast<int32_t>(msg->data/params_.stepper1_gain));
}

void ROSbridge::ROSBridge::stepper2TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
  sprinter_->stepper2SetTargetSteps(static_cast<int32_t>(msg->data/params_.stepper2_gain));
}

void ROSbridge::ROSBridge::servo1TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
  sprinter_->setAngleOfServo1(static_cast<uint16_t>(msg->data/params_.servo1_gain + params_.servo1_offset));
}

void ROSbridge::ROSBridge::servo2TargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
  sprinter_->setAngleOfServo2(static_cast<uint16_t>(msg->data/params_.servo2_gain + params_.servo2_offset));
}

void ROSbridge::ROSBridge::suntrackerCmdCallback(const std_msgs::Empty::ConstPtr& msg)
{
  sprinter_->runSunTracking();
}