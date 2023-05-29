#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>

#define B 0.7             // wheel distance [m]
#define R 0.09            // wheel radius [m]
#define DP 100000000      // DECIMAL_PLACES

class StateEstimation
{
public:
    // Constructor and destructor
    StateEstimation(const ros::Publisher& odom_pub, const ros::Publisher& joint_state_pub);
    ~StateEstimation() = default;

    // Update method
    void update();

    // Callbacks
    void twistCallback(const geometry_msgs::Twist& msg);
    void imuCallback(const sensor_msgs::Imu& msg);
    void stepper1Callback(const std_msgs::Float32& msg);
    void stepper2Callback(const std_msgs::Float32& msg);
    void servo1Callback(const std_msgs::Float32& msg);
    void servo2Callback(const std_msgs::Float32& msg);

private:
    void calculateOdom();
    void publishOdomMsg();
    void publishTFMsg();
    void publishJointStates();
    geometry_msgs::Quaternion createQuaternionMsg(double r, double p, double y);

private:
    // Publishers and broadcasters
    ros::Publisher odom_pub_;
    ros::Publisher joint_state_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;

    nav_msgs::Odometry odom_msg_;
    sensor_msgs::JointState joint_state_msg_;
    geometry_msgs::TransformStamped odom_trans_msg_;

    //
    std_msgs::Float32 stepper1_steps_, stepper2_steps_;
    std_msgs::Float32 servo1_angle_, servo2_angle_;
    double imu_pitch_;
    double lin_vel_, ang_vel_;

    //
    double x_, y_, theta_;
    double left_pos_, right_pos_;

    //
    ros::Time current_time_, last_time_;
};