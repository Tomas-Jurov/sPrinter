#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <sprinter_srvs/SetPoseTarget.h>

class PoseControl
{
public:
  PoseControl(const ros::Publisher& left_speed_pub, const ros::Publisher& right_speed_pub);
  ~PoseControl() = default;

  void Do();

  // Callbacks
  bool targetCallback(sprinter_srvs::SetPoseTarget::Request& req, sprinter_srvs::SetPoseTarget::Response& res);
  void odomCallback(const nav_msgs::Odometry& msg);

private:
  ros::Publisher left_speed_pub_, right_speed_pub_;
};