#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

class TaskManager
{
public:
  TaskManager(const ros::Publisher& vehicle_target_pub, const ros::Publisher& printer_target_pub);
  ~TaskManager() = default;

  void Do();

  // Callbacks
  void vehicleFeedback(const std_msgs::Empty& msg);
  void printerFeedback(const std_msgs::Empty& msg);

private:
  ros::Publisher vehicle_target_pub_, printer_target_pub_;
};