#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "sprinter.h"

namespace ROSbridge
{
  class ROSBridge
  {
  public:
    ROSBridge(const ros::Publisher &wheels_twist_pub
            , const ros::Publisher &stepper1_position_pub
            , const ros::Publisher &stepper2_position_pub
            , const ros::Publisher &servo1_angle_pub
            , const ros::Publisher &servo2_angle_pub
            , const ros::Publisher &suntracker_fb_pub
            , const std::string &port_name
            , const int baud_rate);
    ~ROSBridge() = default;

    void setup();
    void update();

    // Callbacks
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void tiltTargetVelCallback(const std_msgs::Float32::ConstPtr& msg);
    void stepper1SetSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
    void stepper2SetSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
    void stepper1TargetCallback(const std_msgs::Float32::ConstPtr& msg);
    void stepper2TargetCallback(const std_msgs::Float32::ConstPtr& msg);
    void servo1TargetCallback(const std_msgs::Float32::ConstPtr& msg);
    void servo2TargetCallback(const std_msgs::Float32::ConstPtr& msg);
    void suntrackerCmdCallback(const std_msgs::Empty::ConstPtr& msg);

  private:
    ros::Publisher wheels_twist_pub_, stepper1_position_pub_, stepper2_position_pub_,
        servo1_angle_pub_, servo2_angle_pub_, suntracker_fb_pub_;
    
    std::string port_name_;
    int baud_rate_;
    std::unique_ptr<ROSbridge::Sprinter> sprinter_;
    ROSbridge::SpeedOfWheels speed_of_wheels_;
    ROSbridge::Returns returns_;

    geometry_msgs::Twist wheels_twist_msg_;
    std_msgs::Float32 stepper1_position_msg_, stepper2_position_msg_, servo1_angle_msg_, servo2_angle_msg_,
                    suntracker_fb_msg_;
  };
}