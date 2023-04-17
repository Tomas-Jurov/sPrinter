#include "../include/printer_control.h"

PrinterControl::PrinterControl(const ros::Publisher& tilt_pub, const ros::Publisher& stepper1_speed_pub,
                 const ros::Publisher& stepper2_speed_pub, const ros::Publisher& stepper1_target_pub,
                 const ros::Publisher& stepper2_target_pub, const ros::Publisher& servo1_pub,
                 const ros::Publisher& servo2_pub, const ros::ServiceClient& suntracker_client,
                 const ros::ServiceClient& gps_lens_orientation_client)
: tilt_pub_(tilt_pub)
, stepper1_speed_pub_(stepper1_speed_pub)
, stepper2_speed_pub_(stepper2_speed_pub)
, stepper1_target_pub_(stepper1_target_pub)
, stepper2_target_pub_(stepper2_target_pub)
, servo1_pub_(servo1_pub)
, servo2_pub_(servo2_pub)
, suntracker_client_(suntracker_client)
, gps_client_(gps_lens_orientation_client)
, tf_buffer_()
, tf_listener_(tf_buffer_)
{
}

void PrinterControl::Do()
{
}

bool PrinterControl::targetCallback(sprinter_srvs::SetPrinterTarget::Request& req, sprinter_srvs::SetPrinterTarget::Response& res)
{
}

void PrinterControl::stepper1Callback(const std_msgs::Bool& msg)
{
}

void PrinterControl::stepper2Callback(const std_msgs::Bool& msg)
{
}
