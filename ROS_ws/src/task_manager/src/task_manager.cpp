#include "../include/task_manager.h"

TaskManager::TaskManager(const ros::Publisher& pose_cmd_pub, const ros::Publisher& printer_cmd_pub,
                         const ros::Publisher& printer_state_pub, const ros::Publisher& gps_cmd_pub)
  : pose_cmd_pub_(pose_cmd_pub)
  , printer_cmd_pub_(printer_cmd_pub)
  , printer_state_pub_(printer_state_pub)
  , gps_cmd_pub_(gps_cmd_pub)
  , pose_reached_(false)
  , printer_reached_(false)
{
}

void TaskManager::update()
{
  /* ... */

  while (!printer_reached_)
  {
    /* code */
  }
  

  pose_cmd_pub_.publish(pose_cmd_msg_);
  printer_cmd_pub_.publish(printer_cmd_msg_);
  printer_state_pub_.publish(printer_state_msg_);
}

void TaskManager::poseReached(const std_msgs::Bool::ConstPtr& msg)
{
  static int count = 0;
  ROS_INFO("count %d", ++count);
}

void TaskManager::printerReached(const std_msgs::Bool::ConstPtr& msg)
{
  printer_reached_ = msg->data;
}

void TaskManager::gpsFeedback(const std_msgs::Bool::ConstPtr& msg)
{
}
