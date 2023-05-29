#include "../include/printer_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "printer_control");
  ros::NodeHandle nh;
  
  ros::Publisher printer_state_pub = nh.advertise<std_msgs::Int8>("printer_control/state", 1);
  ros::Publisher tilt_pub = nh.advertise<std_msgs::Int8>("tilt/target_vel", 1);
  ros::Publisher stepper1_speed_pub = nh.advertise<std_msgs::Int16>("stepper1/set_speed", 1);
  ros::Publisher stepper2_speed_pub = nh.advertise<std_msgs::Int16>("stepper2/set_speed", 1);
  ros::Publisher stepper1_target_pub = nh.advertise<std_msgs::Float32>("stepper1/target_position", 1);
  ros::Publisher stepper2_target_pub = nh.advertise<std_msgs::Float32>("stepper2/target_position", 1);
  ros::Publisher servo1_pub = nh.advertise<std_msgs::Float32>("servo1/target_angle", 1);
  ros::Publisher servo2_pub = nh.advertise<std_msgs::Float32>("servo2/target_angle", 1);
  ros::Publisher status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("sprinter_status", 50);
  ros::ServiceClient gps_client = nh.serviceClient<sprinter_srvs::GetOrientation>("gps/get_sun_orientation");
  ros::ServiceClient ik_client = nh.serviceClient<sprinter_srvs::GetIkSolution>("ik/get_solution");

  PrinterControl printer_control(printer_state_pub, tilt_pub, stepper1_speed_pub, stepper2_speed_pub,
                                 stepper1_target_pub, stepper2_target_pub, servo1_pub, servo2_pub, status_pub,
                                 gps_client, ik_client);

  ros::Subscriber target_cmd_sub =
      nh.subscribe("target/printer/cmd", 1, &PrinterControl::targetCmdCallback, &printer_control);
  ros::Subscriber target_state_sub =
      nh.subscribe("target/printer/state", 1, &PrinterControl::printerStateCallback, &printer_control);
  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 1, &PrinterControl::jointStateCallback, &printer_control);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    printer_control.update();
    loop_rate.sleep();
  }
}