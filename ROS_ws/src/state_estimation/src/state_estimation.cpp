#include "../include/state_estimation.h"

StateEstimation::StateEstimation(const ros::Publisher& odom_pub, const ros::Publisher& joint_state_pub)
  : odom_pub_(odom_pub), joint_state_pub_(joint_state_pub)
{
  current_time_ = ros::Time::now();
}

void StateEstimation::update()
{
  current_time_ = ros::Time::now();

  calculateOdom();
  publishTFMsg();
  publishOdomMsg();
  publishJointStates();

  last_time_ = ros::Time::now();
}

void StateEstimation::calculateOdom()
{
  // velocity calculation
  double dt = (current_time_ - last_time_).toSec();
  double delta_x = lin_vel_ * cos(theta_) * dt;
  double delta_y = lin_vel_ * sin(theta_) * dt;
  double delta_th = ang_vel_ * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_th;
}

void StateEstimation::publishOdomMsg()
{
  // Create quaternion from yaw data
  geometry_msgs::Quaternion odom_quaternion = createQuaternionMsg(0, 0, theta_);

  // next, we'll publish the odometry message over ROS

  odom_msg_.header.stamp = current_time_;
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";

  // set the position
  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation = odom_quaternion;

  // set the velocity
  odom_msg_.twist.twist.linear.x = lin_vel_;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.angular.z = ang_vel_;

  // publish the message
  odom_pub_.publish(odom_msg_);
}

void StateEstimation::publishTFMsg()
{
  // Create quaternion from yaw data
  geometry_msgs::Quaternion odom_quaternion = createQuaternionMsg(0, 0, theta_);

  // first, we'll publish the transform over tf
  odom_trans_msg_.header.stamp = current_time_;
  odom_trans_msg_.header.frame_id = "odom";
  odom_trans_msg_.child_frame_id = "base_link";

  // set the position
  odom_trans_msg_.transform.translation.x = x_;
  odom_trans_msg_.transform.translation.y = y_;
  odom_trans_msg_.transform.translation.z = 0.0;
  odom_trans_msg_.transform.rotation = odom_quaternion;

  tf_broadcaster_.sendTransform(odom_trans_msg_);
}

void StateEstimation::publishJointStates()
{
  double dt = (current_time_ - last_time_).toSec();
  double left_velocity = (2 * lin_vel_ / R - B * ang_vel_ / R) / 2;
  double right_velocity = (2 * lin_vel_ / R + B * ang_vel_ / R) / 2;

  joint_state_msg_.header.stamp = current_time_;
  joint_state_msg_.name.resize(13);
  joint_state_msg_.position.resize(13);

  left_pos_ += R * left_velocity * dt;
  right_pos_ += R * right_velocity * dt;

  joint_state_msg_.name[0] = "Wheel_left_front";
  joint_state_msg_.name[1] = "Wheel_left_middle";
  joint_state_msg_.name[2] = "Wheel_left_rear";
  joint_state_msg_.name[3] = "Wheel_right_front";
  joint_state_msg_.name[4] = "Wheel_right_middle";
  joint_state_msg_.name[5] = "Wheel_right_rear";
  joint_state_msg_.name[6] = "Boggie_right";
  joint_state_msg_.name[7] = "Boggie_left";
  joint_state_msg_.name[8] = "MainFrame_pitch";
  joint_state_msg_.name[9] = "Lens_Y_axis_trans";
  joint_state_msg_.name[10] = "Lens_X_axis_trans";
  joint_state_msg_.name[11] = "Lens_Y_axis_rot";
  joint_state_msg_.name[12] = "Lens_X_axis_rot";

  joint_state_msg_.position[0] = round(left_pos_ * DP) / DP;
  joint_state_msg_.position[1] = round(left_pos_ * DP) / DP;
  joint_state_msg_.position[2] = round(left_pos_ * DP) / DP;
  joint_state_msg_.position[3] = round(right_pos_ * DP) / DP;
  joint_state_msg_.position[4] = round(right_pos_ * DP) / DP;
  joint_state_msg_.position[5] = round(right_pos_ * DP) / DP;
  joint_state_msg_.position[6] = 0;
  joint_state_msg_.position[7] = 0;
  joint_state_msg_.position[8] = round(imu_pitch_ * DP) / DP;
  joint_state_msg_.position[9] = round(stepper2_steps_.data * DP) / DP;
  joint_state_msg_.position[10] = round(stepper1_steps_.data * DP) / DP;
  joint_state_msg_.position[11] = round(servo2_angle_.data * DP) / DP;
  joint_state_msg_.position[12] = round(servo1_angle_.data * DP) / DP;

  joint_state_pub_.publish(joint_state_msg_);
}

geometry_msgs::Quaternion StateEstimation::createQuaternionMsg(double r, double p, double y)
{
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  geometry_msgs::Quaternion odom_quaternion = tf2::toMsg(q);

  return odom_quaternion;
}

void StateEstimation::twistCallback(const geometry_msgs::Twist& msg)
{
  lin_vel_ = msg.linear.x;
  ang_vel_ = msg.angular.z;
}

void StateEstimation::imuCallback(const sensor_msgs::Imu& msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  imu_pitch_ = pitch;
}

void StateEstimation::stepper1Callback(const std_msgs::Float32& msg)
{
  stepper1_steps_.data = msg.data;
}

void StateEstimation::stepper2Callback(const std_msgs::Float32& msg)
{
  stepper2_steps_.data = msg.data;
}

void StateEstimation::servo1Callback(const std_msgs::Float32& msg)
{
  servo1_angle_.data = msg.data;
}

void StateEstimation::servo2Callback(const std_msgs::Float32& msg)
{
  servo2_angle_.data = msg.data;
}