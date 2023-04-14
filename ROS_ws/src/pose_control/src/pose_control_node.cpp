#include "../include/pose_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_control");
  ros::NodeHandle nh;

  ros::Publisher target_reached_pub = nh.advertise<std_msgs::Empty>("target/vehicle/reached", 1);
  ros::Publisher left_speed_pub = nh.advertise<std_msgs::Int8>("wheels/cmd/left_speed", 1);
  ros::Publisher right_speed_pub = nh.advertise<std_msgs::Int8>("wheels/cmd/right_speed", 1);

  PoseControl pose_control(target_reached_pub, left_speed_pub, right_speed_pub);

  ros::Subscriber target_cmd_sub =
      nh.subscribe("target/vehicle/cmd", 1, &PoseControl::targetCmdCallback, &pose_control);
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, &PoseControl::odomCallback, &pose_control);

  while (nh.ok())
  {
    ros::spinOnce();
  }
}
