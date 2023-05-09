#include "../include/state_estimation.h"

StateEstimation::StateEstimation(const ros::Publisher& odom_pub, const ros::Publisher& joint_state_pub)
: odom_pub_(odom_pub)
, joint_state_pub_(joint_state_pub)
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
  double delta_x = lin_vel_*cos(theta_)*dt;
  double delta_y = lin_vel_*sin(theta_)*dt;
  double delta_th = ang_vel_*dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_th;
}

void StateEstimation::publishOdomMsg()
{
  // Create quaternion from yaw data
  geometry_msgs::Quaternion odom_quaternion = createQuaternionMsgFromYaw(theta_);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  //set the position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quaternion;

  //set the velocity
  odom.twist.twist.linear.x = lin_vel_;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = ang_vel_;

  //publish the message
  odom_pub_.publish(odom);
}

void StateEstimation::publishTFMsg()
{
    // Create quaternion from yaw data
    geometry_msgs::Quaternion odom_quaternion = createQuaternionMsgFromYaw(theta_);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    //set the position
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quaternion;

    tf_broadcaster_.sendTransform(odom_trans);
}

void StateEstimation::publishJointStates()
{
  double dt = (current_time_ - last_time_).toSec();
  double left_velocity = (2*lin_vel_/R - B*ang_vel_/R)/2;
  double right_velocity = (2*lin_vel_/R + B*ang_vel_/R)/2;

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = current_time_;
  joint_state_msg.name.resize(13);
  joint_state_msg.position.resize(13);

  left_pos_ += R*left_velocity*dt;
  right_pos_ += R*right_velocity*dt;

  joint_state_msg.name[0] = "Wheel_left_front";
  joint_state_msg.name[1] = "Wheel_left_middle";
  joint_state_msg.name[2] = "Wheel_left_rear";
  joint_state_msg.name[3] = "Wheel_right_front";
  joint_state_msg.name[4] = "Wheel_right_middle";
  joint_state_msg.name[5] = "Wheel_right_rear";
  joint_state_msg.name[6] = "Boggie_right";
  joint_state_msg.name[7] = "Boggie_left";
  joint_state_msg.name[8] = "MainFrame_pitch";
  joint_state_msg.name[9] = "Lens_Y_axis_trans";
  joint_state_msg.name[10] = "Lens_X_axis_trans";
  joint_state_msg.name[11] = "Lens_Y_axis_rot";
  joint_state_msg.name[12] = "Lens_X_axis_rot";

  joint_state_msg.position[0] = left_pos_;
  joint_state_msg.position[1] = left_pos_;
  joint_state_msg.position[2] = left_pos_;
  joint_state_msg.position[3] = right_pos_;
  joint_state_msg.position[4] = right_pos_;
  joint_state_msg.position[5] = right_pos_;
  joint_state_msg.position[6] = 0;
  joint_state_msg.position[7] = 0;
  joint_state_msg.position[8] = 0;
  joint_state_msg.position[9] = 0;
  joint_state_msg.position[10] = 0;
  joint_state_msg.position[11] = 0;
  joint_state_msg.position[12] = 0;

  joint_state_pub_.publish(joint_state_msg);
}

geometry_msgs::Quaternion StateEstimation::createQuaternionMsgFromYaw(double theta)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  geometry_msgs::Quaternion odom_quaternion = tf2::toMsg(q);

  return odom_quaternion;
}

void StateEstimation::twistCallback(const geometry_msgs::Twist& msg)
{
  lin_vel_ = msg.linear.x;
  ang_vel_ = msg.angular.z;
}

void StateEstimation::gpsCallback(const sensor_msgs::NavSatFix& msg)
{
}

void StateEstimation::imuCallback(const sensor_msgs::Imu& msg)
{
}

void StateEstimation::tiltCmdCallback(const std_msgs::Int8& msg)
{
}

void StateEstimation::stepper1Callback(const std_msgs::Int32& msg)
{
  stepper1_steps_.data = msg.data;
}

void StateEstimation::stepper2Callback(const std_msgs::Int32& msg)
{
  stepper2_steps_.data = msg.data;
}

void StateEstimation::servo1Callback(const std_msgs::Int16& msg)
{
  servo1_angle_.data = msg.data;
}

void StateEstimation::servo2Callback(const std_msgs::Int16& msg)
{
  servo2_angle_.data = msg.data;
}