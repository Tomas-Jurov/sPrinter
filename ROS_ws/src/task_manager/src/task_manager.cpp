#include "../include/task_manager.h"

TaskManager::TaskManager(const ros::ServiceClient& pose_cmd_client, const ros::ServiceClient& printer_cmd_client,
                        const ros::ServiceClient& gps_orientation_client)
: pose_target_client_(pose_cmd_client), printer_target_client_(printer_cmd_client), gps_robot_orientation_client_(gps_orientation_client)
{
}

void TaskManager::Do()
{
  
  /* ... */

  
}
