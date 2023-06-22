#include "../include/pose_control.h"

using namespace PoseControlNS;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_control");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher target_reached_pub = nh.advertise<std_msgs::Bool>("target/pose/reached", 1);

  PoseControl pose_control(cmd_vel_pub, target_reached_pub);
  pose_control.loadParams(nh);

  ros::Subscriber target_cmd_sub = nh.subscribe("target/pose/cmd", 1, &PoseControl::targetCmdCallback, &pose_control);
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, &PoseControl::odomCallback, &pose_control);
  ros::Subscriber stop_sub = nh.subscribe("pose_control/stop", 1, &PoseControl::stopCallback, &pose_control);

  ros::Rate looprate(40);
  while (nh.ok())
  {
    ros::spinOnce();
    looprate.sleep();
  }
}
