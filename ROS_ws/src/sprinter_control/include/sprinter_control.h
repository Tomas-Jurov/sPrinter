#pragma once

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_srvs/Empty.h>

// sPrinter
#include <sprinter_srvs/SetPointArr.h>
#include <sprinter_srvs/SetPose2D.h>

class SprinterControl
{
    public:
			SprinterControl(const ros::Publisher &safety_stop_pub
										, const ros::Publisher &heartbeat_pub
										, const ros::ServiceClient &initialize_client
										, const ros::ServiceClient &set_pose_client
										, const ros::ServiceClient &set_printer_client);
			~SprinterControl() = default;

			void update();

			void taskManagerStatusCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr &msg);

    public:
      static constexpr float HEARTBEAT_INTERVAL = 1.0;	// [s]
    private:
			ros::Publisher safety_stop_pub_, heartbeat_pub_;
			ros::ServiceClient initialize_client_, set_pose_client_, set_printer_client_;
			ros::Time heartbeat_last_;
};