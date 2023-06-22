#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "sprinter.h"

namespace ROSbridge
{
struct Params
{
  std::string port_name;
  int baud_rate;
  float stepper1_gain;
  float stepper2_gain;
  float servo1_gain;
  int servo1_offset;
  float servo2_gain;
  int servo2_offset;
};

class ROSBridge
{
public:
  ROSBridge(const ros::Publisher& wheels_twist_pub, const ros::Publisher& stepper1_position_pub,
            const ros::Publisher& stepper2_position_pub, const ros::Publisher& servo1_angle_pub,
            const ros::Publisher& servo2_angle_pub, const ros::Publisher& suntracker_fb_pub,
            const ros::Publisher& lin_actuator_is_on_point_pub);
  ~ROSBridge() = default;

  void setup();
  void update();
  void loadParams(const ros::NodeHandle& nh);

  // Callbacks
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void tiltTargetVelCallback(const std_msgs::Int8::ConstPtr& msg);
  void stepper1SetSpeedCallback(const std_msgs::Int16::ConstPtr& msg);
  void stepper2SetSpeedCallback(const std_msgs::Int16::ConstPtr& msg);
  void stepper1TargetCallback(const std_msgs::Float32::ConstPtr& msg);
  void stepper2TargetCallback(const std_msgs::Float32::ConstPtr& msg);
  void servo1TargetCallback(const std_msgs::Float32::ConstPtr& msg);
  void servo2TargetCallback(const std_msgs::Float32::ConstPtr& msg);
  void suntrackerCmdCallback(const std_msgs::Empty::ConstPtr& msg);

private:
  template <class T>
  void getParam(const ros::NodeHandle& nh, const std::string& name, T* storage) const;
  void getAndPublishReturns();

private:
  ros::Publisher wheels_twist_pub_, stepper1_position_pub_, stepper2_position_pub_, servo1_angle_pub_,
      servo2_angle_pub_, suntracker_fb_pub_, lin_actuator_is_on_point_pub_;

  std::unique_ptr<ROSbridge::Sprinter> sprinter_;
  ROSbridge::VelocityOfWheels velocity_of_wheels_;
  ROSbridge::Returns returns_;

  geometry_msgs::Twist wheels_twist_msg_;
  std_msgs::Float32 stepper1_position_msg_, stepper2_position_msg_, servo1_angle_msg_, servo2_angle_msg_,
      suntracker_fb_msg_;
  std_msgs::Bool lin_actuator_is_on_point_fb_msg_;
  Params params_;

  static constexpr double R = 0.09;
  static constexpr double B = 0.7;
};
}
