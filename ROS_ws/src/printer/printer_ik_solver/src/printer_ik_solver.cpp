#include "../include/printer_ik_solver.h"

PrinterIKSolver::PrinterIKSolver(const ros::Publisher& status_pub, const ros::ServiceClient& validity_client)
  : status_pub_(status_pub)
  , validity_client_(validity_client)
  , tf_buffer_()
  , tf_listener_(tf_buffer_)
  , move_group_(PLANNING_GROUP)
  , move_group_interface_(PLANNING_GROUP)
{
  /*Constructor*/
  moveit::core::RobotModelConstPtr robot_model = move_group_interface_.getRobotModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  move_group_interface_.setStartStateToCurrentState();

  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group_interface_.setPoseReferenceFrame("base_link");
  robot_state_ = robot_state;
  joint_model_group_ = joint_model_group;
}

/// \brief Calculate inverse kinematics for sPrinter robot lens with enabled approximate solutions.
/// \param desired_pose Pose in base_link frame.
/// \param joint_values_ik Vector of joint values.
/// \return
bool PrinterIKSolver::calculateIK(const geometry_msgs::PoseStamped& desired_pose, std::vector<double>& joint_values_ik)
{
  robot_state_->setToDefaultValues();
  // we may need to do approximate IK
  kinematics::KinematicsQueryOptions o;
  o.return_approximate_solution = true;

  bool ik_found = robot_state_->setFromIK(joint_model_group_, desired_pose.pose, 0.1,
                                          moveit::core::GroupStateValidityCallbackFn(), o);

   std::vector<std::string> joint_names = move_group_interface_.getActiveJoints();

  if (ik_found)
  {
    /*Get the resulting joint state values*/
    robot_state_->copyJointGroupPositions(joint_model_group_, joint_values_ik);

    /*state validity*/
    validity_srv_.request.group_name = "lens_group";
    validity_srv_.request.robot_state.joint_state.position = joint_values_ik;
    validity_srv_.request.robot_state.joint_state.name = joint_names;
    validity_srv_.request.robot_state.joint_state.header.frame_id = "base_link";

    validity_client_.call(validity_srv_);

    if (!validity_srv_.response.valid)
    {
        ik_found = false;
    }

    const Eigen::Affine3d &found_pose = robot_state_->getGlobalLinkTransform("lens_focal");

    //bad attitude IMO, the question is how to check if the found robot pose is suchlike the desired pose
    double diff_trans = 0.01; //1cm max translation
    if (abs(desired_pose.pose.position.x-found_pose.translation().x()) >= diff_trans ||
            abs(desired_pose.pose.position.y-found_pose.translation().y()) >= diff_trans ||
            abs(desired_pose.pose.position.z-found_pose.translation().z()) >= diff_trans)
    {
        ik_found = false;
    }

  }

  if (ik_found)
  {
      ROS_INFO_STREAM(
              "IK solution found\nJoint (" + std::to_string(joint_names.size()) + ") values: "
                                                                                  "\n" +joint_names[0] + ": " + std::to_string(joint_values_ik[0]) +
              "\n" +joint_names[1] + ": " + std::to_string(joint_values_ik[1]) +
              "\n" +joint_names[2] + ": " + std::to_string(joint_values_ik[2]) +
              "\n" +joint_names[3] + ": " + std::to_string(joint_values_ik[3]) +
              "\n" +joint_names[4] + ": " + std::to_string(joint_values_ik[4]));
      publishStatus(LOG_LEVEL_T::OK, "IK solution found");
  }
  else
  {
    publishStatus(LOG_LEVEL_T::ERROR, "Failed to compute IK solution");
    joint_values_ik.empty();
    return false;
  }

  return true;
}

/// \brief Processes IK service request.
/// \param req Desired pose.
/// \param res Array of computed IK joint states if IK solution found.
/// \return True if successful, otherwise false.
bool PrinterIKSolver::calculateIkService(sprinter_srvs::GetIkSolution::Request& req,
                                         sprinter_srvs::GetIkSolution::Response& res)
{
  try
  {
    std::vector<double> joint_states;

    bool success = calculateIK(req.pose, joint_states);

    if (success)
    {
        res.joint_states = joint_states;
    }
    res.success = success;
  }
  catch (...)
  {
    publishStatus(LOG_LEVEL_T::ERROR, "Fatal error when computing IK [try-catch]");
    ROS_ERROR("Fatal error when computing IK [try-catch]");
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