#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define B 1.2 // wheel distance [m]
#define R 0.1 // wheel radius [m]

class StateEstimation
{
public:
  // Constructor and destructor
  StateEstimation(const ros::Publisher& odom_pub);
  ~StateEstimation() = default;

  // Update method
  void update();

  // Callbacks
  void encodersLeftCallback(const std_msgs::Int8::ConstPtr& msg);
  void encodersRightCallback(const std_msgs::Int8::ConstPtr& msg);
  void locationCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void tiltCmdCallback(const std_msgs::Int8::ConstPtr& msg);
  void stepper1Callback(const std_msgs::Int32::ConstPtr& msg);
  void stepper2Callback(const std_msgs::Int32::ConstPtr& msg);
  void servo1Callback(const std_msgs::Int16::ConstPtr& msg);
  void servo2Callback(const std_msgs::Int16::ConstPtr& msg);

private:
  void publishOdomMsg();
  void publishTFMsg();

private:
  // Publishers and broadcasters
  ros::Publisher odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;

  //
  std_msgs::Int8 right_velocity_, left_velocity_;
  std_msgs::Int32 stepper1_steps_, stepper2_steps_;
  std_msgs::Int16 servo1_angle_, servo2_angle_;
  geometry_msgs::Pose2D odom_;

  //
  ros::Time current_time_, last_time_;

};