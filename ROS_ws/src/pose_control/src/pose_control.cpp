#include "../include/pose_control.h"

void PoseControl::Do()
{
    std_msgs::Int8 cmd_left_speed;
    std_msgs::Int8 cmd_right_speed;
    bool target_reached;

    /* ... */

    left_speed_pub_.publish(cmd_left_speed);
    right_speed_pub_.publish(cmd_right_speed);

    
    if (target_reached)
    {
        target_reached_pub_.publish(std_msgs::Empty());
    }
}

void PoseControl::targetCmdCallback(const geometry_msgs::Pose2D &msg)
{

}

void PoseControl::odomCallback(const nav_msgs::Odometry &msg)
{

}
