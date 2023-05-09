#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <sprinter_srvs/GetOrientation.h>

class PrinterControl
{
public:
  PrinterControl(const ros::Publisher& target_reached_pub, const ros::Publisher& tilt_pub,
                 const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                 const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                 const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                 const ros::Publisher& suntracker_pub, const ros::ServiceClient& gps_client);
  ~PrinterControl() = default;

  // Callbacks
  void targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg);
  void targetStateCallback(const std_msgs::Int8::ConstPtr& msg);
  void suntrackerCallback(const std_msgs::Bool::ConstPtr& msg);

  void Do();

private:
  ros::Publisher target_reached_pub_, tilt_pub_, stepper1_speed_pub_, stepper2_speed_pub_, stepper1_target_pub_,
      stepper2_target_pub_, servo1_pub_, servo2_pub_, suntracker_pub_;
  ros::ServiceClient gps_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};