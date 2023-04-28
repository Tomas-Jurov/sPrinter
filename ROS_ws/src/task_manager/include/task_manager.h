#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

class TaskManager
{
public:
  TaskManager(const ros::Publisher& pose_cmd_pub, const ros::Publisher& printer_cmd_pub,
              const ros::Publisher& printer_state_pub, const ros::Publisher& gps_cmd_pub);
  ~TaskManager() = default;

  void Do();

  // Callbacks
  void poseReached(const std_msgs::Bool::ConstPtr& msg);
  void printerReached(const std_msgs::Bool::ConstPtr& msg);
  void gpsFeedback(const std_msgs::Bool::ConstPtr& msg);

private:
  ros::Publisher pose_cmd_pub_, printer_cmd_pub_, printer_state_pub_, gps_cmd_pub_;
};