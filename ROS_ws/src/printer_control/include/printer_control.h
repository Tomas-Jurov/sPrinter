#pragma once

#include "../include/printer_ik_solver.h"

#include "stdio.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sprinter_srvs/GetOrientation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

enum PrinterState
{
    HOME,
    IDLE,
    FAILURE,
    BUSY
};

namespace TaskManagerNS{
    enum TaskManagerState
    {
        HOME,
        IDLE
    };
}

class PrinterControl
{
public:
  PrinterControl(const ros::Publisher& target_reached_pub, const ros::Publisher& tilt_pub,
                 const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                 const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                 const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                 const ros::Publisher& suntracker_pub, const ros::ServiceClient& gps_client);
  ~PrinterControl() = default;

  /*callbacks*/
  void targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg);
  void printerStateCallback(const std_msgs::Int8::ConstPtr& msg);
  void suntrackerCallback(const std_msgs::Bool::ConstPtr& msg);

  /*methods*/
  void update();

private:
  ros::Publisher target_reached_pub_, tilt_pub_, stepper1_speed_pub_, stepper2_speed_pub_, stepper1_target_pub_,
      stepper2_target_pub_, servo1_pub_, servo2_pub_, suntracker_pub_;
  ros::ServiceClient gps_client_;

  tf2_ros::Buffer tf_buffer_, tf_buffer_ik_;
  tf2_ros::TransformListener tf_listener_, tf_listener_ik_;
  std_msgs::Bool target_reached_msg_;
  std_msgs::Float32 tilt_msg_, stepper1_speed_msg_, stepper2_speed_msg_, stepper1_target_msg_, stepper2_target_msg_,
      servo1_msg_, servo2_msg_;
  std_msgs::Empty suntracker_msg_;
  sprinter_srvs::GetOrientation gps_srv_;
private:
  bool lin_actuator_control(double target_angle);

  PrinterIKSolver ik_solver_;

  PrinterState  printer_state_;
  geometry_msgs::Point printing_point_;
  bool go_home_, go_idle_;

  /*fcn*/
  geometry_msgs::Quaternion createQuaternionMsg(double roll, double pitch, double yaw);
};