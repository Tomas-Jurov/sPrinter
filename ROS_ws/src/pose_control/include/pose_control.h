#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

class PoseControl
{
public:
  PoseControl(const ros::Publisher& target_reached_pub, const ros::Publisher& left_speed_pub,
              const ros::Publisher& right_speed_pub);
  ~PoseControl() = default;

  void Do();

  // Callbacks
  void targetCmdCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
  ros::Publisher target_reached_pub_, left_speed_pub_, right_speed_pub_;
};