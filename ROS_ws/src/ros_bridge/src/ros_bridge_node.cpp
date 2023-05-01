#include "../include/ros_bridge.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle nh("~");

  ros::Rate loop_rate(230400);

  ros::Publisher encoders_left_pub = nh.advertise<std_msgs::Int8>("wheels/encoders/left_speed", 1);
  ros::Publisher encoders_right_pub = nh.advertise<std_msgs::Int8>("wheels/encoders/right_speed", 1);
  ros::Publisher encoders_location_pub = nh.advertise<geometry_msgs::Pose2D>("wheels/encoders/location", 1);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  ros::Publisher stepper1_current_pub = nh.advertise<std_msgs::Int32>("stepper1/current_steps", 1);
  ros::Publisher stepper2_current_pub = nh.advertise<std_msgs::Int32>("stepper2/current_steps", 1);
  ros::Publisher servo1_pub = nh.advertise<std_msgs::Int16>("servo1/current_angle", 1);
  ros::Publisher servo2_pub = nh.advertise<std_msgs::Int16>("servo2/current_angle", 1);
  ros::Publisher suntracker_fb_pub = nh.advertise<std_msgs::Bool>("suntracker/done", 1);

  std::string port;
  int baud;
  std::string param_name;
    if (nh.searchParam("port", param_name)) {
    nh.getParam(param_name, port);
  } else {
    ROS_WARN("Parameter 'port_name' not defined");
  }

  if (nh.searchParam("baud", param_name)) {
    nh.getParam(param_name, baud);
  } else {
    ROS_WARN("Parameter 'baudrate' not defined");
  }

  ROSBridge ros_bridge(encoders_left_pub, encoders_right_pub, encoders_location_pub, imu_pub,
                       stepper1_current_pub, stepper2_current_pub, servo1_pub,
                       servo2_pub, suntracker_fb_pub, port, (uint32_t)baud);
  ros_bridge.setup();
  
  ros::Subscriber left_speed_target_sub =
      nh.subscribe("wheels/cmd/left_speed", 1, &ROSBridge::leftSpeedTargetCallback, &ros_bridge);
  ros::Subscriber right_speed_target_sub =
      nh.subscribe("wheels/cmd/right_speed", 1, &ROSBridge::rightSpeedTargetCallback, &ros_bridge);
  ros::Subscriber tilt_speed_target_sub =
      nh.subscribe("tilt/target_speed", 1, &ROSBridge::tiltSpeedTargetback, &ros_bridge);
  ros::Subscriber stepper1_speed_sub =
      nh.subscribe("stepper1/speed", 1, &ROSBridge::stepper1SpeedCallback, &ros_bridge);
  ros::Subscriber stepper2_speed_sub =
      nh.subscribe("stepper2/speed", 1, &ROSBridge::stepper2SpeedCallback, &ros_bridge);
  ros::Subscriber stepper1_target_sub =
      nh.subscribe("stepper1/target_steps", 1, &ROSBridge::stepper1TargetCallback, &ros_bridge);
  ros::Subscriber stepper2_target_sub =
      nh.subscribe("stepper2/target_steps", 1, &ROSBridge::stepper2TargetCallback, &ros_bridge);
  ros::Subscriber servo1_target_sub =
      nh.subscribe("servo1/target_angle", 1, &ROSBridge::servo1TargetCallback, &ros_bridge);
  ros::Subscriber servo2_target_sub =
      nh.subscribe("servo2/target_angle", 1, &ROSBridge::servo2TargetCallback, &ros_bridge);
  ros::Subscriber suntracker_cmd_sub = nh.subscribe("suntracker/do", 1, &ROSBridge::suntrackerCmdCallback, &ros_bridge);

  while (nh.ok())
  {
    ros_bridge.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}