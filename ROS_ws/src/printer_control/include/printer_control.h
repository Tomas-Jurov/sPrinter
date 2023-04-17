#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <sprinter_srvs/SetPrinterTarget.h>
#include <sprinter_srvs/GetOrientation.h>
#include <std_srvs/Trigger.h>

class PrinterControl
{
public:
  PrinterControl(const ros::Publisher& tilt_pub, const ros::Publisher& stepper1_speed_pub,
                 const ros::Publisher& stepper2_speed_pub, const ros::Publisher& stepper1_target_pub,
                 const ros::Publisher& stepper2_target_pub, const ros::Publisher& servo1_pub,
                 const ros::Publisher& servo2_pub, const ros::ServiceClient& suntracker_client,
                 const ros::ServiceClient& gps_client
                 );
  ~PrinterControl() = default;

  // Callbacks
  bool targetCallback(sprinter_srvs::SetPrinterTarget::Request& req, sprinter_srvs::SetPrinterTarget::Response& res);
  void stepper1Callback(const std_msgs::Bool& msg);
  void stepper2Callback(const std_msgs::Bool& msg);
  
  void Do();

private:
  ros::Publisher tilt_pub_, stepper1_speed_pub_, stepper2_speed_pub_, stepper1_target_pub_,
      stepper2_target_pub_, servo1_pub_, servo2_pub_;
  ros::ServiceClient suntracker_client_, gps_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};