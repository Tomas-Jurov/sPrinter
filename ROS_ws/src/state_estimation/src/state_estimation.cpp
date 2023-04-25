#include "../include/state_estimation.h"

StateEstimation::StateEstimation(const ros::Publisher& odom_pub) : odom_pub_(odom_pub)
{
  current_time_ = ros::Time::now();
}

void StateEstimation::update()
{
  current_time_ = ros::Time::now();

  publishTFMsg();
  publishOdomMsg();
}

void StateEstimation::publishOdomMsg()
{
  // velocity calculation
  double vel_x = R*(right_velocity_.data + left_velocity_.data)/2;
  double vel_theta = R*(right_velocity_.data - left_velocity_.data)/B;

  // Create quaternion from yaw data
  geometry_msgs::Quaternion odom_quaternion = createQuaternionMsgFromYaw(odom_.theta);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  //set the position
  odom.pose.pose.position.x = odom_.x;
  odom.pose.pose.position.y = odom_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quaternion;

  //set the velocity
  odom.twist.twist.linear.x = vel_x;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = vel_theta;

  //publish the message
  odom_pub_.publish(odom);
}

void StateEstimation::publishTFMsg()
{
  // Create quaternion from yaw data
  geometry_msgs::Quaternion odom_quaternion = createQuaternionMsgFromYaw(odom_.theta);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  //set the position
  odom_trans.transform.translation.x = odom_.x;
  odom_trans.transform.translation.y = odom_.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quaternion;

  tf_broadcaster_.sendTransform(odom_trans);
}

geometry_msgs::Quaternion StateEstimation::createQuaternionMsgFromYaw(double &theta)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  geometry_msgs::Quaternion odom_quaternion = tf2::toMsg(q);

  return odom_quaternion;
}

void StateEstimation::encodersLeftCallback(const std_msgs::Int8& msg)
{
  left_velocity_ = msg;
}

void StateEstimation::encodersRightCallback(const std_msgs::Int8::ConstPtr& msg)
{
  right_velocity_ = msg;
}

void StateEstimation::locationCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  odom_ = msg;
}

void StateEstimation::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
}

void StateEstimation::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
}

void StateEstimation::tiltCmdCallback(const std_msgs::Int8::ConstPtr& msg)
{
}

void StateEstimation::stepper1Callback(const std_msgs::Int32::ConstPtr& msg)
{
  stepper1_steps_ = msg;
}

void StateEstimation::stepper2Callback(const std_msgs::Int32::ConstPtr& msg)
{
  stepper2_steps_ = msg;
}

void StateEstimation::servo1Callback(const std_msgs::Int16::ConstPtr& msg)
{
  servo1_angle_ = msg;
}

void StateEstimation::servo2Callback(const std_msgs::Int16::ConstPtr& msg)
{
  servo2_angle_ = msg;
}