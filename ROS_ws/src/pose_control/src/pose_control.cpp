#include "../include/pose_control.h"

PoseControl::PoseControl(const ros::Publisher& target_reached_pub, const ros::Publisher& cmd_vel_pub)
  : target_reached_pub_(target_reached_pub), cmd_vel_pub_(cmd_vel_pub)
{
}

void PoseControl::update()
{
  /* ... */

  cmd_vel_pub_.publish(cmd_vel_msg_);

  target_reached_msg_.data = target_reached_;
  target_reached_pub_.publish(target_reached_msg_);
}

void PoseControl::targetCmdCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
}

void PoseControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
}
