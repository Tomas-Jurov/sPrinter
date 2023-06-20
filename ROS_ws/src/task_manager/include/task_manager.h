#pragma once

// C++
#include <string>
#include <queue>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_srvs/Empty.h>

// sPrinter
#include <sprinter_srvs/SetPointArr.h>
#include <sprinter_srvs/SetPose2D.h>

typedef diagnostic_msgs::DiagnosticStatus LOG_LEVEL_T;

namespace TaskManagerNS
{
enum State
{
  START = 0,
  INITIALIZING,
  ROBOT_READY,
  ROBOT_MOVING,
  PRINTER_BUSY,
  PRINTER_IDLE,
  STOPPING,
  RESETTING
};

enum PrinterState
{
  HOME = 0,
  IDLE,
  FAILURE
};

class TaskManager
{
public:
  TaskManager(const ros::Publisher& pose_cmd_pub, const ros::Publisher& printer_cmd_pub,
              const ros::Publisher& printer_state_pub, ros::Publisher& pose_stop_pub, 
              const ros::Publisher& gps_do_pub, const ros::Publisher& gps_reset_pub, 
              const ros::Publisher& status_pub, const ros::Publisher& reset_odom_pub);
  ~TaskManager() = default;

  void update();

  // Input callbacks
  bool safetyStopCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool resetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void heartbeatCallback(const std_msgs::Empty::ConstPtr& msg);
  bool initializeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool poseTaskCallback(sprinter_srvs::SetPose2D::Request& req, sprinter_srvs::SetPose2D::Response& res);
  bool printerTaskCallback(sprinter_srvs::SetPointArr::Request& req, sprinter_srvs::SetPointArr::Response& res);
  // Feedback callbacks
  void gpsProcessingCallback(const std_msgs::Bool::ConstPtr& msg);
  void poseControlCallback(const std_msgs::Bool::ConstPtr& msg);
  void printerControlCallback(const std_msgs::Int8::ConstPtr& msg);

public:
  static constexpr float WATCHDOG_TIMEOUT = 5;  // [s]

private:
  bool watchdogCheck();
  void safetyStop();
  void reset();
  void setState(TaskManagerNS::State state);
  void publishStatus(const int8_t logger_level, const std::string& message);
  void publishPoseTarget();
  void publishPrinterTargetCmd();
  void publishPrinterTargetState(const PrinterState state);
  void publishPoseStop();

  void clearPoseTask();
  void clearPrinterTask();

  bool connected_;
  TaskManagerNS::State state_;
  ros::Time watchdog_last_;

  ros::Publisher pose_cmd_pub_, printer_cmd_pub_, printer_state_pub_, pose_stop_pub_, gps_do_pub_, gps_reset_pub_, 
                status_pub_, reset_odom_pub_;

  std_msgs::Int8 printer_state_msg_;
  diagnostic_msgs::DiagnosticStatus status_msg_;

  std::unique_ptr<geometry_msgs::Pose2D> pose_task_ptr_;
  std::queue<geometry_msgs::Point> printer_task_;
};
}
