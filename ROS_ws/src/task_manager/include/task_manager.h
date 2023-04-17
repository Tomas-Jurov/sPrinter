#pragma once

#include <ros/ros.h>
#include <sprinter_srvs/SetPoseTarget.h>
#include <sprinter_srvs/SetPrinterTarget.h>
#include <std_srvs/Trigger.h>

class TaskManager
{
public:
  TaskManager(const ros::ServiceClient& pose_cmd_client, const ros::ServiceClient& printer_cmd_client,
              const ros::ServiceClient& gps_orientation_client);
  ~TaskManager() = default;

  void Do();

private:
  ros::ServiceClient pose_target_client_, printer_target_client_, gps_robot_orientation_client_;
};