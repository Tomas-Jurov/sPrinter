#include "../include/task_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_manager");
  ros::NodeHandle nh;

  ros::ServiceClient pose_target_client = nh.serviceClient<sprinter_srvs::SetPoseTarget>("prose_control/target");
  ros::ServiceClient printer_target_client = nh.serviceClient<sprinter_srvs::SetPrinterTarget>("printer_control/target");
  ros::ServiceClient gps_robot_orientation_client = nh.serviceClient<std_srvs::Trigger>("gps/robot_global_orientation");
  
  TaskManager task_manager(pose_target_client, printer_target_client, gps_robot_orientation_client);

  while (nh.ok())
  {
    ros::spinOnce();
  }
}
