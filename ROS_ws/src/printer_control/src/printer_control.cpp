#include "../include/printer_control.h"

PrinterControl::PrinterControl(const ros::Publisher& target_reached_pub, const ros::Publisher& tilt_pub,
                               const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                               const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                               const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                               const ros::Publisher& suntracker_pub, const ros::ServiceClient& gps_client)
  : target_reached_pub_(target_reached_pub)
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

void PrinterControl::update()
{
  // just an example how to publish messages using member variables
  target_reached_pub_.publish(target_reached_msg_);
  tilt_pub_.publish(tilt_msg_);
  stepper1_speed_pub_.publish(stepper1_speed_msg_);
  stepper2_speed_pub_.publish(stepper2_speed_msg_);
  stepper1_target_pub_.publish(stepper1_target_msg_);
  stepper2_target_pub_.publish(stepper2_target_msg_);
  servo1_pub_.publish(servo1_msg_);
  servo2_pub_.publish(servo2_msg_);
  suntracker_pub_.publish(suntracker_msg_);
  gps_client_.call(gps_srv_);
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
