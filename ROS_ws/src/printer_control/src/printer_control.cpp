#include "../include/printer_control.h"

using TaskManagerNS::TaskManagerState;

PrinterControl::PrinterControl(const ros::Publisher& target_reached_pub, const ros::Publisher& tilt_pub,
                               const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                               const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                               const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                               const ros::Publisher& suntracker_pub, const ros::ServiceClient& gps_client)
  : target_reached_pub_(target_reached_pub)
  , tilt_pub_(tilt_pub)
  , stepper1_speed_pub_(stepper1_speed_pub)
  , stepper2_speed_pub_(stepper2_speed_pub)
  , stepper1_target_pub_(stepper1_target_pub)
  , stepper2_target_pub_(stepper2_target_pub)
  , servo1_pub_(servo1_pub)
  , servo2_pub_(servo2_pub)
  , suntracker_pub_(suntracker_pub)
  , gps_client_(gps_client)
  , tf_buffer_()
  , tf_listener_(tf_buffer_)
  , tf_listener_ik_(tf_buffer_ik_)
  , ik_solver_("lens_group")
  , printer_state_(HOME)
  , go_home_(true)
  , go_idle_(true)
  {
    //Constructor
    ROS_INFO_STREAM("Printer control constructor");

    //create quaternion msg
    geometry_msgs::Quaternion my_quaternion = createQuaternionMsg(5, 0, 0);

    // Define the desired end-effector pose in lens_focal_static_frame frame
    geometry_msgs::PoseStamped desired_pose;
    desired_pose.header.frame_id = "lens_focal_static_frame";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.pose.position.x = 0;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 0;
    desired_pose.pose.orientation.x = my_quaternion.x;
    desired_pose.pose.orientation.y = my_quaternion.y;
    desired_pose.pose.orientation.z =  my_quaternion.z;
    desired_pose.pose.orientation.w = my_quaternion.w;

    geometry_msgs::PoseStamped desired_pose_in_base_link_ = tf_buffer_ik_.transform(
            desired_pose, "base_link");

    ROS_INFO_STREAM("desired_pose_in_base_link_ \nx: " << desired_pose_in_base_link_.pose.position.x <<
                                               "\ny: "<< desired_pose_in_base_link_.pose.position.y<<
                                               "\nz: " << desired_pose_in_base_link_.pose.position.z);

    std::vector<double> joint_values;
    if (ik_solver_.calculateIK(desired_pose_in_base_link_, joint_values))
    {
//        ROS_INFO_STREAM("Ik solution found");
    }
    else
    {
//        ROS_INFO_STREAM("Cannot find solution");
    }
}

void PrinterControl::update()
{

}

void PrinterControl::targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ROS_INFO_STREAM("targetCmdCallback");
    if (printer_state_ == IDLE)
    {
        printing_point_ = *msg;
        ROS_INFO_STREAM("New printing_point: \nx: " << printing_point_.x << "\ny: " << printing_point_.y << "\nz: " << printing_point_.z) ;
    }
    else
    {
        ROS_WARN("Cannot update \"printing_point_\", printer is not in state IDLE");
    }

}

void PrinterControl::printerStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if (msg->data == TaskManagerState::HOME)
    {
        if (printer_state_ != HOME)
        {
            ROS_INFO_STREAM("printerStateCallback: HOME");
            go_home_ = true;
        }
        else
        {
            ROS_INFO_STREAM("Printer already HOME");
        }

    }
    else if (msg->data == TaskManagerState::IDLE)
    {
        /*I do not want to receive IDLE if printer is BUSY*/
        if (printer_state_ != IDLE && printer_state_ != BUSY)
        {
            ROS_INFO_STREAM("printerStateCallback: IDLE");
            go_idle_ = true;
        }
        else if (printer_state_ != BUSY)
        {
            ROS_INFO_STREAM("Printer already IDLE");
        }
    }
}

void PrinterControl::suntrackerCallback(const std_msgs::Bool::ConstPtr& msg)
{

}

geometry_msgs::Quaternion PrinterControl::createQuaternionMsg(double roll_deg, double pitch_deg, double yaw_deg)
{
    tf2::Quaternion my_quaternion;
    my_quaternion.setRPY(roll_deg/180.0*M_PI, pitch_deg/180.0*M_PI, yaw_deg/180.0*M_PI);
    geometry_msgs::Quaternion my_quaternion_msg = tf2::toMsg(my_quaternion);
    return my_quaternion_msg;
}

bool PrinterControl::lin_actuator_control(double target_angle)
{
  double current_angle, error;
  double roll, pitch, yaw;
  double u, Kp = 200;
  geometry_msgs::TransformStamped transformStamped;
  std_msgs::Int8 msg;

  // Read transformation if possible
  try
  {
    transformStamped = tf_buffer_.lookupTransform("base_link", "main_frame_1", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // Transform quaternion to RPY angles
  tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  // P controller
  current_angle = pitch;
  error = target_angle - current_angle;
  u = Kp * (error);

  // Publish msg
  msg.data = (int8_t)u;
  tilt_pub_.publish(msg);

  if (error < 0.017)  // 0.017 = 1 deg
  {
    return true;
  }
  else
  {
    return false;
  }
}
