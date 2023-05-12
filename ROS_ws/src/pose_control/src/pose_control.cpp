#include "../include/pose_control.h"

PoseControl::PoseControl(const ros::Publisher& target_reached_pub, const ros::Publisher& cmd_vel_pub)
  : target_reached_pub_(target_reached_pub), cmd_vel_pub_(cmd_vel_pub)
{
}

void PoseControl::Do()
{
  geometry_msgs::Twist cmd_vel_msg;
  std_msgs::Bool target_reached_msg;
  bool target_reached;

  /* ... */

  cmd_vel_pub_.publish(cmd_vel_msg);

  target_reached_msg.data = target_reached;
  target_reached_pub_.publish(target_reached_msg);
}

void PoseControl::targetCmdCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
}

void PoseControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
}
