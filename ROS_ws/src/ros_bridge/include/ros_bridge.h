#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include "sprinter.h"

class ROSBridge
{
public:
  ROSBridge(ros::NodeHandle* nh);
  ~ROSBridge() = default;

  void setup();
  void update();

  // Callbacks
  void leftSpeedTargetCallback(const std_msgs::Int8& msg);
  void rightSpeedTargetCallback(const std_msgs::Int8& msg);
  void tiltSpeedTargetback(const std_msgs::Int8& msg);
  void stepper1SpeedCallback(const std_msgs::Int16& msg);
  void stepper2SpeedCallback(const std_msgs::Int16& msg);
  void stepper1TargetCallback(const std_msgs::Int32& msg);
  void stepper2TargetCallback(const std_msgs::Int32& msg);
  void servo1TargetCallback(const std_msgs::Int16& msg);
  void servo2TargetCallback(const std_msgs::Int16& msg);
  void suntrackerCmdCallback(const std_msgs::Empty& msg);

private:
  bool getParameters();

private:
  ros::NodeHandle *nh_;
  ros::Publisher encoders_left_pub_, encoders_right_pub_, imu_pub_,
                 stepper1_current_pub_, stepper2_current_pub_, 
                 servo1_pub_, servo2_pub_, suntracker_fb_pub_;
  ros::Subscriber left_speed_target_sub_, right_speed_target_sub_, tilt_speed_target_sub_, 
                  stepper1_speed_sub_, stepper2_speed_sub_, stepper1_target_sub_,
                  stepper2_target_sub_, servo1_target_sub_, servo2_target_sub_, suntracker_cmd_sub_; 

  std::string port_name_;
  int baud_rate_;
  std::unique_ptr<sprinter::Sprinter> sprinter_;
  sprinter::SpeedOfWheels speed_of_wheels_;
  sprinter::Returns returns_;
};