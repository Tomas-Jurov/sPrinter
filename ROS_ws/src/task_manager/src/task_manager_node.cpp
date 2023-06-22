#include "../include/task_manager.h"

using namespace TaskManagerNS;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_manager");
  ros::NodeHandle nh;

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("target/pose/cmd", 1);
  ros::Publisher printer_cmd_pub = nh.advertise<geometry_msgs::Point>("target/printer/cmd", 1);
  ros::Publisher printer_state_pub = nh.advertise<std_msgs::Int8>("target/printer/state", 1);
  ros::Publisher pose_stop_pub = nh.advertise<std_msgs::Empty>("pose_control/stop", 1);
  ros::Publisher gps_do_pub = nh.advertise<std_msgs::Empty>("gps/global_orientation/do", 1);
  ros::Publisher gps_reset_pub = nh.advertise<std_msgs::Empty>("gps/global_orientation/reset", 1);
  ros::Publisher reset_odom_pub = nh.advertise<std_msgs::Empty>("reset_odometry", 1);
  ros::Publisher status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("sprinter_status", 10);

  TaskManager task_manager(pose_pub, printer_cmd_pub, printer_state_pub, pose_stop_pub, gps_do_pub, gps_reset_pub,
                           status_pub, reset_odom_pub);

  ros::Subscriber pose_sub =
      nh.subscribe("target/pose/reached", 1, &TaskManager::TaskManager::poseControlCallback, &task_manager);
  ros::Subscriber printer_sub =
      nh.subscribe("/printer_control/state", 1, &TaskManager::printerControlCallback, &task_manager);
  ros::Subscriber gps_sub =
      nh.subscribe("gps/global_orientation/done", 1, &TaskManager::gpsProcessingCallback, &task_manager);
  ros::Subscriber heartbeat_sub =
      nh.subscribe("sprinter_control/heartbeat", 1, &TaskManager::heartbeatCallback, &task_manager);
  ros::ServiceServer stop_server = nh.advertiseService("safety_stop", &TaskManager::safetyStopCallback, &task_manager);
  ros::ServiceServer reset_server = nh.advertiseService("reset", &TaskManager::resetCallback, &task_manager);
  ros::ServiceServer init_server =
      nh.advertiseService("task_manager/initialize", &TaskManager::initializeCallback, &task_manager);
  ros::ServiceServer pose_target_server =
      nh.advertiseService("task_manager/set_task/pose", &TaskManager::poseTaskCallback, &task_manager);
  ros::ServiceServer printer_target_server =
      nh.advertiseService("task_manager/set_task/printer", &TaskManager::printerTaskCallback, &task_manager);

  ros::Rate looprate(100);
  while (nh.ok())
  {
    ros::spinOnce();
    task_manager.update();
    looprate.sleep();
  }
}
