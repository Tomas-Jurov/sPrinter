#ifndef PRINTER_IK_SOLVER_H
#define PRINTER_IK_SOLVER_H

#define PLANNING_GROUP "lens_group"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sprinter_srvs/GetIkSolution.h>
#include <../../../../devel/include/sprinter_srvs/GetIkSolution.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

typedef diagnostic_msgs::DiagnosticStatus LOG_LEVEL_T;

class PrinterIKSolver
{
public:
  PrinterIKSolver(const ros::Publisher& status_pub);
  //    ~PrinterIKSolver() = default;

  bool calculateIkService(sprinter_srvs::GetIkSolution::Request& req, sprinter_srvs::GetIkSolution::Response& res);
  bool calculateIK(const geometry_msgs::PoseStamped& desired_pose, std::vector<double>& joint_values);

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;

  ros::Publisher status_pub_;
  diagnostic_msgs::DiagnosticStatus status_msg_;
  ros::ServiceServer get_ik_service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void publishStatus(const int8_t logger_level, const std::string& message);
};

#endif  // PRINTER_IK_SOLVER_H
