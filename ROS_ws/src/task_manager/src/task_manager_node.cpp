#include "../include/task_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_manager");
  ros::NodeHandle nh;

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("target/pose/cmd", 1);
  ros::Publisher printer_cmd_pub = nh.advertise<geometry_msgs::Point>("target/printer/cmd", 1);
  ros::Publisher printer_state_pub = nh.advertise<std_msgs::Int8>("target/printer/state", 1);
  ros::Publisher gps_pub = nh.advertise<std_msgs::Empty>("gps/robot_global_orientation/cmd", 1);

  TaskManager task_manager(pose_pub, printer_cmd_pub, printer_state_pub, gps_pub);

  ros::Subscriber pose_sub = nh.subscribe("target/pose/reached", 1, &TaskManager::poseReached, &task_manager);
  ros::Subscriber printer_sub = nh.subscribe("target/printer/reached", 1, &TaskManager::printerReached, &task_manager);
  ros::Subscriber gps_sub = nh.subscribe("gps/robot_global_orientation/done", 1, &TaskManager::gpsFeedback, &task_manager);

  ros::Rate looprate(100);
  while (nh.ok())
  {
    ros::spinOnce();
    task_manager.Do();
    looprate.sleep();
  }
}
