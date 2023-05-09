#include "../include/task_manager.h"

TaskManager::TaskManager(const ros::Publisher& pose_cmd_pub, const ros::Publisher& printer_cmd_pub,
                        const ros::Publisher& printer_state_pub, const ros::Publisher& gps_cmd_pub)
: pose_cmd_pub_(pose_cmd_pub), printer_cmd_pub_(printer_cmd_pub), printer_state_pub_(printer_state_pub), gps_cmd_pub_(gps_cmd_pub)
{
}

void TaskManager::Do()
{
  geometry_msgs::Pose2D pose_cmd_msg;
  geometry_msgs::Point printer_cmd_msg;
  std_msgs::Int8 printer_state_msg;

  /* ... */

  pose_cmd_pub_.publish(pose_cmd_msg);
  printer_cmd_pub_.publish(printer_cmd_msg);
  printer_state_pub_.publish(printer_state_msg);
}

void TaskManager::poseReached(const std_msgs::Bool::ConstPtr& msg)
{
}

void TaskManager::printerReached(const std_msgs::Bool::ConstPtr& msg)
{
}

void TaskManager::gpsFeedback(const std_msgs::Bool::ConstPtr& msg)
{

}
