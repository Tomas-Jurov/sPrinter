#include "../include/pose_control.h"

PoseControl::PoseControl(const ros::Publisher& left_speed_pub,
                         const ros::Publisher& right_speed_pub)
: left_speed_pub_(left_speed_pub), right_speed_pub_(right_speed_pub)
{
}

void PoseControl::Do()
{
  std_msgs::Int8 cmd_left_speed;
  std_msgs::Int8 cmd_right_speed;
  bool target_reached;

  /* ... */

  left_speed_pub_.publish(cmd_left_speed);
  right_speed_pub_.publish(cmd_right_speed);


}

bool PoseControl::targetCallback(sprinter_srvs::SetPoseTarget::Request& req, sprinter_srvs::SetPoseTarget::Response& res)
{
  return true;
}

void PoseControl::odomCallback(const nav_msgs::Odometry& msg)
{
  
}
