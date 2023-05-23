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


class PrinterIKSolver
{
public:
    PrinterIKSolver();
//    ~PrinterIKSolver() = default;

    bool calculateIkService(sprinter_srvs::GetIkSolution::Request& req,
                            sprinter_srvs::GetIkSolution::Response& res);
    bool calculateIK(const geometry_msgs::PoseStamped& desired_pose, std::vector<double>& joint_values);

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const std::string planning_group_ = "lens_group";
    const std::string end_effector_link_ = "lens_focal_work_frame";
    const std::string reference_frame_ = "lens_focal_static_frame";
    const std::string robot_description_ = "robot_description";

    ros::ServiceServer get_ik_service_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

};

#endif // PRINTER_IK_SOLVER_H
