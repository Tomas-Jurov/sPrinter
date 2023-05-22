#include "../include/printer_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "printer_control");
  ros::NodeHandle nh;

  ros::Publisher target_reached_pub = nh.advertise<std_msgs::Bool>("target/printer/reached", 1);
  ros::Publisher tilt_pub = nh.advertise<std_msgs::Float32>("tilt/target_vel", 1);
  ros::Publisher stepper1_speed_pub = nh.advertise<std_msgs::Float32>("stepper1/set_speed", 1);
  ros::Publisher stepper2_speed_pub = nh.advertise<std_msgs::Float32>("stepper2/set_speed", 1);
  ros::Publisher stepper1_target_pub = nh.advertise<std_msgs::Float32>("stepper1/target_position", 1);
  ros::Publisher stepper2_target_pub = nh.advertise<std_msgs::Float32>("stepper2/target_position", 1);
  ros::Publisher servo1_pub = nh.advertise<std_msgs::Float32>("servo1/target_angle", 1);
  ros::Publisher servo2_pub = nh.advertise<std_msgs::Float32>("servo2/target_angle", 1);
  ros::Publisher suntracker_pub = nh.advertise<std_msgs::Empty>("suntracker/do", 1);
  ros::ServiceClient gps_client = nh.serviceClient<sprinter_srvs::GetOrientation>("gps/get_sun_orientation");

  PrinterControl printer_control(target_reached_pub, tilt_pub, stepper1_speed_pub, stepper2_speed_pub,
                                 stepper1_target_pub, stepper2_target_pub, servo1_pub, servo2_pub, suntracker_pub,
                                 gps_client);

  ros::Subscriber target_cmd_sub =
      nh.subscribe("target/printer/cmd", 1, &PrinterControl::targetCmdCallback, &printer_control);
  ros::Subscriber target_state_sub =
      nh.subscribe("target/printer/state", 1, &PrinterControl::printerStateCallback, &printer_control);
  ros::Subscriber suntracker_sub =
      nh.subscribe("suntracker/done", 1, &PrinterControl::suntrackerCallback, &printer_control);

  ros::Rate looprate(100);
  while (nh.ok())
  {
    ros::spinOnce();
    printer_control.update();
    looprate.sleep();
  }
}