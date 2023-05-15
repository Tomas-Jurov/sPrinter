#include "../include/ros_bridge.h"
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle nh;

  // Publlished topics
  ros::Publisher wheels_twist_pub = nh.advertise<geometry_msgs::Twist>("wheels/twist", 1);
  ros::Publisher stepper1_position_pub = nh.advertise<std_msgs::Float32>("stepper1/current_position", 1);
  ros::Publisher stepper2_position_pub = nh.advertise<std_msgs::Float32>("stepper2/current_position", 1);
  ros::Publisher servo1_angle_pub = nh.advertise<std_msgs::Float32>("servo1/current_angle", 1);
  ros::Publisher servo2_angle_pub = nh.advertise<std_msgs::Float32>("servo2/current_angle", 1);
  ros::Publisher suntracker_fb_pub = nh.advertise<std_msgs::Bool>("suntracker/done", 1);

  // Parameters
  std::string port_name;
  if (!nh.getParam("/mcu_serial/port", port_name))
  {
    ROS_ERROR("Could not find '/mcu_serial/port' parameter!");
  }
  
  int baud_rate(0);
  if (!nh.getParam("/mcu_serial/baud", baud_rate))
  {
    ROS_ERROR("Could not find '/mcu_serial/baud' parameter!");
  } 
  ROS_INFO_STREAM("LOADED PARAMETERS: " << baud_rate << " " << port_name);

  ROSbridge::ROSBridge ros_bridge(wheels_twist_pub, stepper1_position_pub, stepper2_position_pub, servo1_angle_pub,
                      servo2_angle_pub, suntracker_fb_pub, port_name, baud_rate);

  // Subscribed topics
  ros::Subscriber cmd_vel_sub 
    = nh.subscribe("cmd_vel", 1, &ROSbridge::ROSBridge::cmdVelCallback, &ros_bridge);
  ros::Subscriber tilt_target_vel_sub 
    = nh.subscribe("tilt/target_vel", 1, &ROSbridge::ROSBridge::tiltTargetVelCallback, &ros_bridge);
  ros::Subscriber stepper1_set_speed_sub
    = nh.subscribe("stepper1/set_speed", 1, &ROSbridge::ROSBridge::stepper1SetSpeedCallback, &ros_bridge);
  ros::Subscriber stepper2_set_speed_sub 
    = nh.subscribe("stepper2/set_speed", 1, &ROSbridge::ROSBridge::stepper2SetSpeedCallback, &ros_bridge);
  ros::Subscriber stepper1_target_sub 
    = nh.subscribe("stepper1/target_position", 1, &ROSbridge::ROSBridge::stepper1TargetCallback, &ros_bridge);
  ros::Subscriber stepper2_target_sub 
    = nh.subscribe("stepper2/target_position", 1, &ROSbridge::ROSBridge::stepper2TargetCallback, &ros_bridge);
  ros::Subscriber servo1_target_sub 
    = nh.subscribe("servo1/target_angle", 1, &ROSbridge::ROSBridge::servo1TargetCallback, &ros_bridge);
  ros::Subscriber servo2_target_sub 
    = nh.subscribe("servo2/target_angle", 1, &ROSbridge::ROSBridge::servo2TargetCallback, &ros_bridge);
  ros::Subscriber suntracker_cmd_sub 
    = nh.subscribe("suntracker/do", 1, &ROSbridge::ROSBridge::suntrackerCmdCallback, &ros_bridge);

  ros_bridge.setup();

  ros::Rate loop_rate(230400);
  while (nh.ok())
  {
    ros_bridge.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}