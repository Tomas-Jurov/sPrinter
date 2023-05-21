#include "../include/sprinter_control.h"

SprinterControl::SprinterControl(const ros::Publisher &safety_stop_pub
															, const ros::Publisher &heartbeat_pub
															, const ros::ServiceClient &initialize_client
															, const ros::ServiceClient &set_pose_client
															, const ros::ServiceClient &set_printer_client)
	: safety_stop_pub_(safety_stop_pub)
	, heartbeat_pub_(heartbeat_pub)
	, initialize_client_(initialize_client)
	, set_pose_client_(set_pose_client)
	, set_printer_client_(set_printer_client)
	, heartbeat_last_(ros::Time::now())
{
}

void SprinterControl::update()
{
	if (ros::Duration(ros::Time::now() - heartbeat_last_).toSec() > HEARTBEAT_INT)
		{
			heartbeat_pub_.publish(std_msgs::Empty());
			heartbeat_last_ = ros::Time::now();
		}
}

void SprinterControl::taskManagerStatusCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr &msg)
{
	ros::console::Level log_level;
	switch (msg->level)
	{
		case diagnostic_msgs::DiagnosticStatus::OK 		: log_level = ros::console::levels::Info; break;
		case diagnostic_msgs::DiagnosticStatus::WARN  : log_level = ros::console::levels::Warn; break;
		case diagnostic_msgs::DiagnosticStatus::ERROR : log_level = ros::console::levels::Error; break;
		default 																			: log_level = ros::console::levels::Debug; break;
	}

	ROS_LOG(log_level, ROSCONSOLE_DEFAULT_NAME, "%s: %s", msg->name.c_str(), msg->message.c_str());
}