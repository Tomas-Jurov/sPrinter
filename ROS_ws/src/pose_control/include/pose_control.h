#pragma once

#include <algorithm>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

namespace PoseControlNS
{
struct Params
{
  float max_lin_speed;
  float max_ang_speed;
  float target_position_tolerance;
  float target_orientation_tolerance;
  float look_ahead_distance;
  float ang_vel_gain;
};

class PoseControl
{
public:
  PoseControl(const ros::Publisher& cmd_vel_pub, const ros::Publisher& target_reached_pub);
  ~PoseControl() = default;

  void update();
  void loadParams(const ros::NodeHandle& nh);

  // Callbacks
  void targetCmdCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void stopCallback(const std_msgs::Empty::ConstPtr& msg);

private:
  ros::Publisher cmd_vel_pub_, target_reached_pub_;
  geometry_msgs::Twist cmd_vel_msg_;
  std_msgs::Bool target_reached_msg_;

  Params params_;
  geometry_msgs::Pose2D target_, pose_;

  bool target_reached_ = true;

private:
  template <class T>
  void getParam(const ros::NodeHandle& nh, const std::string& name, T* storage) const
  {
    if (!nh.getParam(name, *storage))
      ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
  };

  template <typename T>
  int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  };

  double euclidean_distance(double x1, double y1, double x2, double y2)
  {
    tf::Vector3 v1(x1, y1, 0.0);
    tf::Vector3 v2(x2, y2, 0.0);
    return v1.distance(v2);
  };
};
}
