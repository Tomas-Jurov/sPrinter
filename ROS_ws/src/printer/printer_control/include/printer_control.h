#pragma once

#define ERR_TRESHOLD_ANG 0.017 //1deg tolerance
#define ERR_TRESHOLD_POS 0.005 //5mm
#define KP_GAIN 200
#define KI_GAIN 10
#define PRINTING_TIMEOUT 600 //10min
#define IDLE_TIMEOUT 5 //5s
#define PRINTING_FRAME "lens_focal_static_frame"

#include "printer_state.h"

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sprinter_srvs/GetOrientation.h>
#include <../../../../devel/include/sprinter_srvs/GetOrientation.h>
#include <sprinter_srvs/GetIkSolution.h>
#include <../../../../devel/include/sprinter_srvs/GetIkSolution.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

struct actuatorStruct{
    bool is_set = false;
    bool is_on_pos = false;
} ;

class PrinterControl
{
public:
  PrinterControl(const ros::Publisher& target_reached_pub, const ros::Publisher& tilt_pub,
                 const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                 const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                 const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                 const ros::Publisher& suntracker_pub, const ros::ServiceClient& gps_client,
                 const ros::ServiceClient& ik_client);
  ~PrinterControl() = default;

  /*callbacks*/
  void targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg);
  void printerStateCallback(const std_msgs::Int8::ConstPtr& msg);
  void suntrackerCallback(const std_msgs::Bool::ConstPtr& msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /*methods*/
  void update();

private:
  ros::Publisher printer_state_pub_, tilt_pub_, stepper1_speed_pub_, stepper2_speed_pub_, stepper1_target_pub_,
      stepper2_target_pub_, servo1_pub_, servo2_pub_, suntracker_pub_;
  ros::ServiceClient gps_client_;
  ros::ServiceClient ik_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std_msgs::Bool target_reached_msg_;
  std_msgs::Float32  stepper1_target_msg_, stepper2_target_msg_, servo1_msg_, servo2_msg_;
  std_msgs::Int8 tilt_msg_;
  std_msgs::Int16 stepper1_speed_msg_, stepper2_speed_msg_;
  std_msgs::Empty suntracker_msg_;
  sprinter_srvs::GetOrientation gps_srv_;
  sprinter_srvs::GetIkSolution ik_srv_;

private:


  PrinterState  printer_state_;
  geometry_msgs::Point printing_point_;
  bool go_home_, go_idle_, go_print_, need_initialize_, printing_pose_found_, need_go_home_;
  std::vector<double> joint_positions_, joint_positions_target_;
  actuatorStruct servo1, servo2, stepper1, stepper2, lin_actuator;
  geometry_msgs::Quaternion quaternion_world_sun;
  geometry_msgs::PoseStamped printing_pose_;
  ros::Time printing_start_timestamp_, idle_start_timestamp_, lin_actuator_last_time_;
  double integrator_ = 0.0;

  /*fcn*/
  void servo2Update(bool condition);
  void servo2Update();
  void servo1Update(bool condition);
  void servo1Update();
  void linActuatorUpdate();
  void stepper2Update(bool condition);
  void stepper2Update();
  void stepper1Update(bool condition);
  void stepper1Update();
  void resetActuatorsStruct();
  bool servosOnPos();
  bool steppersOnPos();
  bool isInIdle2();
  bool lin_actuator_control(double target_angle);
  void goHome();
  void goIdle();
  void goPrint();
};