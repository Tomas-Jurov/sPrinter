#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

class ROSBridge
{
public:
  ROSBridge(const ros::Publisher& encoders_left_pub, const ros::Publisher& encoders_right_pub,
            const ros::Publisher& encoders_location_pub, const ros::Publisher& imu_pub,
            const ros::Publisher& stepper1_current_pub, const ros::Publisher& stepper2_current_pub,
            const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub, const ros::Publisher& suntracker_fb_pub);
  ~ROSBridge() = default;

  void Do();

  // Callbacks
  void wheelsSpeedTargetCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void tiltSpeedTargetCallback(const std_msgs::Int8::ConstPtr& msg);
  void stepper1SpeedCallback(const std_msgs::Int16::ConstPtr& msg);
  void stepper2SpeedCallback(const std_msgs::Int16::ConstPtr& msg);
  void stepper1TargetCallback(const std_msgs::Int32::ConstPtr& msg);
  void stepper2TargetCallback(const std_msgs::Int32::ConstPtr& msg);
  void servo1TargetCallback(const std_msgs::Int16::ConstPtr& msg);
  void servo2TargetCallback(const std_msgs::Int16::ConstPtr& msg);
  void suntrackerCmdCallback(const std_msgs::Empty::ConstPtr& msg);

private:
  ros::Publisher encoders_left_pub_, encoders_right_pub_, encoders_location_pub_, imu_pub_, stepper1_current_pub_,
                 stepper2_current_pub_, servo1_pub_, servo2_pub_, suntracker_fb_pub_;
};