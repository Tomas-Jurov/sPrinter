#ifndef PRINTER_IK_SOLVER_H
#define PRINTER_IK_SOLVER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>


class PrinterIKSolver
{
public:
    PrinterIKSolver(const std::string& group_name);

    bool calculateIK(const geometry_msgs::PoseStamped& desired_pose, std::vector<double>& joint_values);

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const std::string end_effector_link_ = "lens_focal_work_frame";
    const std::string reference_frame_ = "lens_focal_static_frame";
    const std::string robot_description_ = "robot_description";

};

#endif // PRINTER_IK_SOLVER_H
