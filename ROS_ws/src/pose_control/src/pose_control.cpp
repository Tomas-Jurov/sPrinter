#include "../include/pose_control.h"

namespace PoseControlNS
{
PoseControl::PoseControl(const ros::Publisher& cmd_vel_pub, const ros::Publisher& target_reached_pub)
  : cmd_vel_pub_(cmd_vel_pub), target_reached_pub_(target_reached_pub)
{
}

void PoseControl::loadParams(const ros::NodeHandle& nh)
{
  getParam(nh, "pose_control/max_lin_speed", &params_.max_lin_speed);
  getParam(nh, "pose_control/max_ang_speed", &params_.max_ang_speed);
  getParam(nh, "pose_control/target_position_tolerance", &params_.target_position_tolerance);
  getParam(nh, "pose_control/target_orientation_tolerance", &params_.target_orientation_tolerance);
  getParam(nh, "pose_control/look_ahead_distance", &params_.look_ahead_distance);
  getParam(nh, "pose_control/ang_vel_gain", &params_.ang_vel_gain);
}

void PoseControl::update()
{
  static double distance_to_go, turn_error, r, heading_error;

  if (target_reached_)
  {
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
  }
  else
  {
    distance_to_go = euclidean_distance(target_.x, target_.y, pose_.x, pose_.y);
    if (distance_to_go > params_.target_position_tolerance)
    {
      turn_error = std::atan2((target_.y - pose_.y), (target_.x - pose_.x)) - pose_.theta;
      if (std::abs(turn_error) > M_PI)
      {
        turn_error = -1 * sgn(turn_error) * (2 * M_PI - std::abs(turn_error));
      }

      if (abs(turn_error) > M_PI_2)
      {
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = sgn(turn_error) * params_.max_ang_speed;
      }
      else
      {  // Pure Pursuit
        cmd_vel_msg_.linear.x = params_.max_lin_speed;
        cmd_vel_msg_.angular.z =
            std::min(params_.ang_vel_gain * abs(turn_error), static_cast<double>(params_.max_ang_speed)) *
            sgn(turn_error);
      }
    }
    else
    {
      ROS_DEBUG_ONCE("position reached");
      heading_error = target_.theta - pose_.theta;
      if (heading_error > M_PI || heading_error < -M_PI)
      {
        heading_error = -1 * sgn(heading_error) * (2 * M_PI - std::abs(heading_error));
      }

      if (std::abs(heading_error) > params_.target_orientation_tolerance)
      {
        cmd_vel_msg_.angular.z = sgn(heading_error) * params_.max_ang_speed;
        cmd_vel_msg_.linear.x = 0.0;
      }

      else
      {
        ROS_DEBUG("TARGET REACHED");
        target_reached_ = true;
      }
    }
  }

  cmd_vel_pub_.publish(cmd_vel_msg_);
  if (target_reached_msg_.data != target_reached_)
  {
    target_reached_msg_.data = target_reached_;
    target_reached_pub_.publish(target_reached_msg_);
  }
}

void PoseControl::targetCmdCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  ROS_DEBUG("new target obtained");
  target_reached_ = false;
  target_.x = msg->x;
  target_.y = msg->y;
  target_.theta = msg->theta;
}

void PoseControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose_.theta = yaw;
  pose_.x = msg->pose.pose.position.x;
  pose_.y = msg->pose.pose.position.y;
  update();
}

void PoseControl::stopCallback(const std_msgs::Empty::ConstPtr& msg)
{
  target_reached_ = true;
}
}
