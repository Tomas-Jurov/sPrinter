#include "../include/printer_control.h"

PrinterControl::PrinterControl(const ros::Publisher& target_reached_pub, const ros::Publisher& tilt_pub,
                               const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                               const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                               const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                               const ros::Publisher& suntracker_pub, const ros::ServiceClient& gps_client)
  : target_reached_pub_()
  , tilt_pub_(tilt_pub)
  , stepper1_speed_pub_(stepper1_speed_pub)
  , stepper2_speed_pub_(stepper2_speed_pub)
  , stepper1_target_pub_(stepper1_target_pub)
  , stepper2_target_pub_(stepper2_target_pub)
  , servo1_pub_(servo1_pub)
  , servo2_pub_(servo2_pub)
  , suntracker_pub_(suntracker_pub)
  , gps_client_(gps_client)
  , tf_buffer_()
  , tf_listener_(tf_buffer_)
{
}

void PrinterControl::Do()
{
}

void PrinterControl::targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg)
{
}

void PrinterControl::targetStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
}

void PrinterControl::suntrackerCallback(const std_msgs::Bool::ConstPtr& msg)
{
}
