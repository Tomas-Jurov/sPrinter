#include "../include/ros_bridge.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle nh("~");

  ros::Rate loop_rate(230400);

  ROSBridge ros_bridge(&nh);
  ros_bridge.setup();

  while (nh.ok())
  {
    ros_bridge.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}