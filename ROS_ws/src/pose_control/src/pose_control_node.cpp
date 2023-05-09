#include "../include/pose_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_control");
  ros::NodeHandle nh;

  ros::Publisher target_reached_pub = nh.advertise<std_msgs::Bool>("target/pose/reached", 1);
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("wheels/cmd_vel", 1);
  
  PoseControl pose_control(target_reached_pub, cmd_vel_pub);

  ros::Subscriber target_cmd_sub =
      nh.subscribe("target/pose/cmd", 1, &PoseControl::targetCmdCallback, &pose_control);
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, &PoseControl::odomCallback, &pose_control);

  ros::Rate looprate(100);
  while (nh.ok())
  {
    ros::spinOnce();
    pose_control.Do();
    looprate.sleep();
  }
}
