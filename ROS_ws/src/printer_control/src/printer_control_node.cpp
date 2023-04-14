#include "../include/printer_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "printer_control");
  ros::NodeHandle nh;

  ros::Publisher target_reached_pub = nh.advertise<std_msgs::Empty>("target/printer/reached", 1);
  ros::Publisher tilt_pub = nh.advertise<std_msgs::Int8>("tilt/target_speed", 1);
  ros::Publisher stepper1_speed_pub = nh.advertise<std_msgs::Int16>("stepper1/speed", 1);
  ros::Publisher stepper2_speed_pub = nh.advertise<std_msgs::Int16>("stepper2/speed", 1);
  ros::Publisher stepper1_target_pub = nh.advertise<std_msgs::Int32>("stepper1/target_steps", 1);
  ros::Publisher stepper2_target_pub = nh.advertise<std_msgs::Int32>("stepper2/target_steps", 1);
  ros::Publisher servo1_pub = nh.advertise<std_msgs::Int16>("servo1/target_angle", 1);
  ros::Publisher servo2_pub = nh.advertise<std_msgs::Int16>("servo2/target_angle", 1);
  ros::Publisher suntracker_pub = nh.advertise<std_msgs::Empty>("suntracker/do", 1);

  PrinterControl printer_control(target_reached_pub, tilt_pub, stepper1_speed_pub, stepper2_speed_pub,
                                 stepper1_target_pub, stepper2_target_pub, servo1_pub, servo2_pub, suntracker_pub);

  ros::Subscriber target_cmd_sub =
      nh.subscribe("target/printer/cmd", 1, &PrinterControl::targetCmdCallback, &printer_control);
  ros::Subscriber stepper1_sub = nh.subscribe("stepper1/idle", 1, &PrinterControl::stepper1Callback, &printer_control);
  ros::Subscriber stepper2_sub = nh.subscribe("stepper2/idle", 1, &PrinterControl::stepper2Callback, &printer_control);
  ros::Subscriber suntracker_sub =
      nh.subscribe("suntracker/done", 1, &PrinterControl::suntrackerCallback, &printer_control);

  while (nh.ok())
  {
    ros::spinOnce();
  }
}