#include "../include/ros_bridge.h"

ROSBridge::ROSBridge(const ros::Publisher& encoders_left_pub, const ros::Publisher& encoders_right_pub,
            const ros::Publisher& encoders_location_pub, const ros::Publisher& imu_pub,
            const ros::Publisher& stepper1_current_pub, const ros::Publisher& stepper2_current_pub,
            const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub, const ros::Publisher& suntracker_fb_pub,
            const std::string& port_name, const uint32_t baud_rate)
            : encoders_left_pub_(encoders_left_pub)
            , encoders_right_pub_(encoders_right_pub)
            , encoders_location_pub_(encoders_location_pub)
            , imu_pub_(imu_pub)
            , stepper1_current_pub_(stepper1_current_pub)
            , stepper2_current_pub_(stepper2_current_pub)
            , servo1_pub_(servo1_pub)
            , servo2_pub_(servo2_pub)
            , suntracker_fb_pub_(suntracker_fb_pub)
            , sprinter_(std::make_unique<sprinter::Sprinter>(port_name, baud_rate)) 
            {
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
    sprinter_->setSpeedOfWheels(&speed_of_wheels_);
    sprinter_->readReturns(&returns_);
    // std::cout << returns_.left_grp_speed << " " << returns_.right_grp_speed << " " << returns_.pose.theta << 
    // " " << returns_.pose.x << " " << returns_.pose.y << " " << returns_.servo1_current_angle << " " << returns_.servo2_current_angle <<
    // " " << returns_.stepper1_current_steps << " " << returns_.stepper2_current_steps << " " << returns_.suntracker_done << std::endl;
    std_msgs::Int8 left_speed;
    std_msgs::Int8 right_speed;
    geometry_msgs::Pose2D location;
    std_msgs::Int32 stepper1_current_steps;
    std_msgs::Int32 stepper2_current_steps;
    std_msgs::Int16 servo1_current_angle;
    std_msgs::Int16 servo2_current_angle;
    std_msgs::Bool suntracker_done;
    left_speed.data = returns_.left_grp_speed;
    right_speed.data = returns_.right_grp_speed;
    location.theta = returns_.pose.theta;
    location.x = returns_.pose.x;
    location.y = returns_.pose.y;
    stepper1_current_steps.data = returns_.stepper1_current_steps;
    stepper2_current_steps.data = returns_.stepper2_current_steps;
    servo1_current_angle.data = returns_.servo1_current_angle;
    servo2_current_angle.data = returns_.servo2_current_angle;
    suntracker_done.data = returns_.suntracker_done;

    encoders_left_pub_.publish(left_speed);
    encoders_right_pub_.publish(right_speed);  
    encoders_location_pub_.publish(location);
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
