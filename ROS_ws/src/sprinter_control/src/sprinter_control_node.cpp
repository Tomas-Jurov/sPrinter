#include "../include/sprinter_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sprinter_control");
  ros::NodeHandle nh;

  ros::Publisher safety_stop_pub = nh.advertise<std_msgs::Empty>("sprinter_control/safety_stop", 1);
  ros::Publisher heartbeat_pub = nh.advertise<std_msgs::Empty>("sprinter_control/heartbeat", 1);
  ros::ServiceClient initialize_client = nh.serviceClient<std_srvs::Empty>("task_manager/initialize");
  ros::ServiceClient set_pose_client = nh.serviceClient<sprinter_srvs::SetPose2D>("task_manager/set_task/pose");
  ros::ServiceClient set_printer_client = nh.serviceClient<sprinter_srvs::SetPointArr>("task_manager/set_task/printer");

  SprinterControl sprinter_control(safety_stop_pub, heartbeat_pub, initialize_client, set_pose_client,
                                   set_printer_client);

  ros::Subscriber task_manager_status_sub =
      nh.subscribe("task_manager/status", 10, &SprinterControl::taskManagerStatusCallback, &sprinter_control);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    sprinter_control.update();
    loop_rate.sleep();
  }
}