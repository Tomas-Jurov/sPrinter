#include "../include/state_estimation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimation");
  ros::NodeHandle nh;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

  StateEstimation state_estimation(odom_pub);

  ros::Subscriber encoders_left_sub =
      nh.subscribe("wheels/encoders/left_speed", 1, &StateEstimation::encodersLeftCallback, &state_estimation);
  ros::Subscriber encoders_right_sub =
      nh.subscribe("wheels/encoders/right_speed", 1, &StateEstimation::encodersRightCallback, &state_estimation);
  ros::Subscriber location_sub =
      nh.subscribe("wheels/encoders/location", 1, &StateEstimation::locationCallback, &state_estimation);
  ros::Subscriber gps_sub = nh.subscribe("gps/fix", 1, &StateEstimation::gpsCallback, &state_estimation);
  ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, &StateEstimation::imuCallback, &state_estimation);
  ros::Subscriber tilt_sub = nh.subscribe("tilt/cmd_speed", 1, &StateEstimation::tiltCmdCallback, &state_estimation);
  ros::Subscriber stepper1_sub =
      nh.subscribe("stepper1/current_steps", 1, &StateEstimation::stepper1Callback, &state_estimation);
  ros::Subscriber stepper2_sub =
      nh.subscribe("stepper2/current_steps", 1, &StateEstimation::stepper2Callback, &state_estimation);
  ros::Subscriber servo1_sub =
      nh.subscribe("servo1/current_angle", 1, &StateEstimation::servo1Callback, &state_estimation);
  ros::Subscriber servo2_sub =
      nh.subscribe("servo2/current_angle", 1, &StateEstimation::servo2Callback, &state_estimation);

  while (nh.ok())
  {
    ros::spinOnce();
  }
}