#include "../include/ros_bridge.h"

ROSBridge::ROSBridge(ros::NodeHandle *nh)
: nh_(nh)
, encoders_left_pub_(nh_->advertise<std_msgs::Int8>("wheels/encoders/left_speed", 1))
, encoders_right_pub_(nh_->advertise<std_msgs::Int8>("wheels/encoders/right_speed", 1))
, imu_pub_(nh_->advertise<sensor_msgs::Imu>("imu/data", 1))
, stepper1_current_pub_(nh_->advertise<std_msgs::Int32>("stepper1/current_steps", 1))
, stepper2_current_pub_(nh_->advertise<std_msgs::Int32>("stepper2/current_steps", 1))
, servo1_pub_(nh_->advertise<std_msgs::Int16>("servo1/current_angle", 1))
, servo2_pub_(nh_->advertise<std_msgs::Int16>("servo2/current_angle", 1))
, suntracker_fb_pub_(nh_->advertise<std_msgs::Bool>("suntracker/done", 1))
, left_speed_target_sub_(nh_->subscribe("wheels/cmd/left_speed", 1, &ROSBridge::leftSpeedTargetCallback, this))
, right_speed_target_sub_(nh_->subscribe("wheels/cmd/right_speed", 1, &ROSBridge::rightSpeedTargetCallback, this))
, tilt_speed_target_sub_(nh_->subscribe("tilt/target_speed", 1, &ROSBridge::tiltSpeedTargetback, this))
, stepper1_speed_sub_(nh_->subscribe("stepper1/speed", 1, &ROSBridge::stepper1SpeedCallback, this))
, stepper2_speed_sub_(nh_->subscribe("stepper2/speed", 1, &ROSBridge::stepper2SpeedCallback, this))
, stepper1_target_sub_(nh_->subscribe("stepper1/target_steps", 1, &ROSBridge::stepper1TargetCallback, this))
, stepper2_target_sub_(nh_->subscribe("stepper2/target_steps", 1, &ROSBridge::stepper2TargetCallback, this))
, servo1_target_sub_(nh_->subscribe("servo1/target_angle", 1, &ROSBridge::servo1TargetCallback, this))
, servo2_target_sub_(nh_->subscribe("servo2/target_angle", 1, &ROSBridge::servo2TargetCallback, this))
, suntracker_cmd_sub_(nh_->subscribe("suntracker/do", 1, &ROSBridge::suntrackerCmdCallback, this))
{
  if (getParameters() == 0)
    sprinter_ = std::make_unique<sprinter::Sprinter>(port_name_, baud_rate_);
}

bool ROSBridge::getParameters()
{
  if (!nh_->getParam("port", port_name_))
  {
    ROS_ERROR("Couldn not find 'port' parameter!");
    return 1;
  }

  if (!nh_->getParam("baud", baud_rate_))
  {
    ROS_ERROR("Couldn not find 'baud' parameter!");
    return 1;
  }

  return 0;
}

void ROSBridge::setup()
{
  sprinter_->connect();
}

void ROSBridge::update()
{
  if (sprinter_.get() != nullptr)
  {
    speed_of_wheels_.left_speed = 5;
    speed_of_wheels_.right_speed = -15;
    sprinter_->setSpeedOfWheels(speed_of_wheels_);
    sprinter_->readReturns(&returns_);
    // std::cout << returns_.left_grp_speed << " " << returns_.right_grp_speed << " " << returns_.pose.theta << 
    // " " << returns_.pose.x << " " << returns_.pose.y << " " << returns_.servo1_current_angle << " " << returns_.servo2_current_angle <<
    // " " << returns_.stepper1_current_steps << " " << returns_.stepper2_current_steps << " " << returns_.suntracker_done << std::endl;
    std_msgs::Int8 left_speed;
    std_msgs::Int8 right_speed;
    std_msgs::Int32 stepper1_current_steps;
    std_msgs::Int32 stepper2_current_steps;
    std_msgs::Int16 servo1_current_angle;
    std_msgs::Int16 servo2_current_angle;
    std_msgs::Bool suntracker_done;
    left_speed.data = returns_.left_grp_speed;
    right_speed.data = returns_.right_grp_speed;
    stepper1_current_steps.data = returns_.stepper1_current_steps;
    stepper2_current_steps.data = returns_.stepper2_current_steps;
    servo1_current_angle.data = returns_.servo1_current_angle;
    servo2_current_angle.data = returns_.servo2_current_angle;
    suntracker_done.data = returns_.suntracker_done;

    encoders_left_pub_.publish(left_speed);
    encoders_right_pub_.publish(right_speed);  
    servo1_pub_.publish(servo1_current_angle);
    servo2_pub_.publish(servo2_current_angle);
    stepper1_current_pub_.publish(stepper1_current_steps);
    stepper2_current_pub_.publish(stepper2_current_steps);
    suntracker_fb_pub_.publish(suntracker_done);
  }
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

void ROSBridge::suntrackerCmdCallback(const std_msgs::Empty& msg)
{
}
