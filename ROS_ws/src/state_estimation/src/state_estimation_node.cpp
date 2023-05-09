#include "../include/state_estimation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimation");
  ros::NodeHandle nh;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);

  StateEstimation state_estimation(odom_pub, joint_state_pub);

  ros::Subscriber twist_sub =
      nh.subscribe("wheels/twist", 1, &StateEstimation::twistCallback, &state_estimation);
  ros::Subscriber gps_sub = nh.subscribe("gps/fix", 1, &StateEstimation::gpsCallback, &state_estimation);
  ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, &StateEstimation::imuCallback, &state_estimation);
  ros::Subscriber stepper1_sub =
      nh.subscribe("stepper1/current_steps", 1, &StateEstimation::stepper1Callback, &state_estimation);
  ros::Subscriber stepper2_sub =
      nh.subscribe("stepper2/current_steps", 1, &StateEstimation::stepper2Callback, &state_estimation);
  ros::Subscriber servo1_sub =
      nh.subscribe("servo1/current_angle", 1, &StateEstimation::servo1Callback, &state_estimation);
  ros::Subscriber servo2_sub =
      nh.subscribe("servo2/current_angle", 1, &StateEstimation::servo2Callback, &state_estimation);

	// set the rate of TF publishing
  ros::Rate looprate(100);
  while (nh.ok())
  {
    state_estimation.update();
    ros::spinOnce();
		looprate.sleep();
  }
}