#include "../include/printer_ik_solver.h"

PrinterIKSolver::PrinterIKSolver(const ros::Publisher& status_pub) :
        status_pub_(status_pub),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        move_group_(PLANNING_GROUP),
        move_group_interface_(PLANNING_GROUP)
{
    /*Constructor*/
    moveit::core::RobotModelConstPtr robot_model = move_group_interface_.getRobotModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    move_group_interface_.setStartStateToCurrentState();

    const moveit::core::JointModelGroup* joint_model_group =
            move_group_interface_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_state_ = robot_state;
    joint_model_group_ = joint_model_group;

}

/// \brief Calculate inverse kinematics for sPrinter robot lens.
/// \param desired_pose Pose in base_link frame.
/// \param joint_values_ik Vector of joint values.
/// \return
bool PrinterIKSolver::calculateIK(const geometry_msgs::PoseStamped& desired_pose, std::vector<double>& joint_values_ik)
{
    robot_state_->setToDefaultValues();
    // we may need to do approximate IK
    kinematics::KinematicsQueryOptions o;
    o.return_approximate_solution = true;

    bool ik_found = robot_state_->setFromIK(joint_model_group_, desired_pose.pose, 0.1,  moveit::core::GroupStateValidityCallbackFn(), o);

    if (ik_found)
    {
        /*Get the resulting joint state values*/
        robot_state_->copyJointGroupPositions(joint_model_group_, joint_values_ik);

        ROS_INFO_STREAM("IK solution found\nJoint values: "
                        "\njoint 9 MainFrame_pitch: " + std::to_string(joint_values_ik[0]) +
                        "\njoint 10 Lens_Y_axis_trans: " + std::to_string(joint_values_ik[1]) +
                        "\njoint 11 Lens_X_axis_trans: " + std::to_string(joint_values_ik[2]) +
                        "\njoint 12 Lens_Y_axis_rot: " + std::to_string(joint_values_ik[3]) +
                        "\njoint 13 Lens_X_axis_rot: " + std::to_string(joint_values_ik[4]));
        publishStatus(LOG_LEVEL_T::OK, "IK solution found");
    }
    else
    {
        publishStatus(LOG_LEVEL_T::ERROR, "Failed to compute IK solution");
        return false;
    }

    return true;
}

/// \brief Processes IK service request.
/// \param req Desired pose.
/// \param res Array of computed IK joint states if IK solution found.
/// \return True if successful, otherwise false.
bool PrinterIKSolver::calculateIkService(sprinter_srvs::GetIkSolution::Request& req, sprinter_srvs::GetIkSolution::Response& res)
{
    try
    {
        calculateIK(req.pose, res.joint_states );
    }
    catch(...)
    {
        publishStatus(LOG_LEVEL_T::ERROR, "Fatal error when computing IK [try-catch]");
        ROS_ERROR( "Fatal error when computing IK [try-catch]");
    }


    return true;
}

/// \brief Publish status to status topic.
/// \param logger_level Level as OK, WARN, ERROR
/// \param message
void PrinterIKSolver::publishStatus(const int8_t logger_level, const std::string& message)
{
    status_msg_.level = logger_level;
    status_msg_.name = "Printer IK";
    status_msg_.message = message;

    if (!status_msg_.message.empty())
        status_pub_.publish(status_msg_);
}