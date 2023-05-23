#include "../include/printer_control.h"
#include "../include/printer_joint_positions.h"

PrinterControl::PrinterControl(const ros::Publisher& printer_state_pub, const ros::Publisher& tilt_pub,
                               const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                               const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                               const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                               const ros::Publisher& suntracker_pub, const ros::ServiceClient& gps_client,
                               const ros::ServiceClient& ik_client)
  : printer_state_pub_(printer_state_pub)
  , tilt_pub_(tilt_pub)
  , stepper1_speed_pub_(stepper1_speed_pub)
  , stepper2_speed_pub_(stepper2_speed_pub)
  , stepper1_target_pub_(stepper1_target_pub)
  , stepper2_target_pub_(stepper2_target_pub)
  , servo1_pub_(servo1_pub)
  , servo2_pub_(servo2_pub)
  , suntracker_pub_(suntracker_pub)
  , gps_client_(gps_client)
  , ik_client_(ik_client)
  , tf_buffer_()
  , tf_listener_(tf_buffer_)
  , printer_state_(IDLE)
  , go_home_(false), go_idle_(false), go_print_(false)
  , need_initialize_(true)
  , printing_pose_found_(false)
  {
    //Constructor

}

void PrinterControl::update()
{
    if (go_home_) goHome();

    if (go_idle_) goIdle();

    if (go_print_) goPrint();

    if (printer_state_ == PRINTING &&
            printing_start_timestamp_.sec-ros::Time::now().sec > PRINTING_TIMEOUT)
    {
        printer_state_ = IDLE;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
    }

    if (printer_state_ == IDLE &&
            idle_start_timestamp_.sec-ros::Time::now().sec > IDLE_TIMEOUT)
    {
        joint_positions_target_ = joint_positions_idle2_;
        go_idle_ = true;
    }

}

void PrinterControl::goPrint()
{
    if (printer_state_ != BUSY) printer_state_ = BUSY;
    if (!printing_pose_found_)
    {
        printing_pose_.pose.orientation = quaternion_world_sun;
        printing_pose_.pose.position = printing_point_;
        printing_pose_.header.frame_id = PRINTING_FRAME;
        printing_pose_.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped printing_pose_in_base_link_ = tf_buffer_.transform(
                printing_pose_, "base_link");

        ROS_INFO_STREAM("desired_pose_in_base_link_ \nx: " << printing_pose_in_base_link_.pose.position.x <<
                                                           "\ny: "<< printing_pose_in_base_link_.pose.position.y<<
                                                           "\nz: " << printing_pose_in_base_link_.pose.position.z);

        ik_srv_.request.pose = printing_pose_in_base_link_;
        ik_client_.call(ik_srv_);
        if (!ik_srv_.response.joint_states.size())
        {
            printer_state_ = FAILURE;
            std_msgs::Int8 msg;
            msg.data = printer_state_;
            printer_state_pub_.publish(msg);
            joint_positions_target_ = joint_positions_; //important
        }
    }

    stepper2Update();
    stepper1Update();
    linActuatorUpdate();
    servo2Update(steppersOnPos());
    servo1Update(steppersOnPos());

    // on printing pos
    if (servosOnPos() && steppersOnPos() && lin_actuator.is_on_pos)
    {
        printer_state_ = PRINTING;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
        go_print_ = false;
        resetActuatorsStruct();
        printing_start_timestamp_ = ros::Time::now();
    }
}

void PrinterControl::goIdle()
{
    if (need_initialize_)
    {
        printer_state_ = INIT;
        gps_client_.call(gps_srv_);
        quaternion_world_sun = gps_srv_.response.orientation;
        need_initialize_ = false;
        printer_state_ = BUSY;
    }
    else if (printer_state_ != BUSY) printer_state_ = BUSY;

    stepper2Update();
    stepper1Update();
    linActuatorUpdate();
    servo2Update(steppersOnPos());
    servo1Update(steppersOnPos());

    // on idle pos
    if (servosOnPos() && steppersOnPos() && lin_actuator.is_on_pos)
    {
        printer_state_ = IDLE;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
        go_idle_ = false;
        resetActuatorsStruct();
        idle_start_timestamp_ = ros::Time::now();
    }

}

void PrinterControl::goHome()
{
    if (!need_initialize_) need_initialize_ = true;
    if (printer_state_ != BUSY) printer_state_ = BUSY;
    servo2Update();
    servo1Update();
    linActuatorUpdate();
    stepper2Update(servosOnPos());
    stepper1Update(servosOnPos());

    // on home pos
    if (servosOnPos() && steppersOnPos() && lin_actuator.is_on_pos)
    {
        printer_state_ = HOME;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
        go_home_ = false;
        resetActuatorsStruct();
    }
}

void PrinterControl::servo1Update(bool condition)
{
    // servo1 Lens_X_axis_rot
    if (abs(joint_positions_home_[4]-joint_positions_[4]) >= MAX_ERR_ANG)
    {
        if (!servo1.is_set && condition)
        {
            std_msgs::Float32 msg;
            msg.data = joint_positions_home_[4];
            servo1_pub_.publish(msg);
            servo1.is_set = true;
        }
    }
    else servo1.is_on_pos = true;
}

void PrinterControl::servo1Update()
{
    servo1Update(true);
}

void PrinterControl::servo2Update(bool condition)
{
    // servo2 Lens_Y_axis_rot
    if (abs(joint_positions_home_[3]-joint_positions_[3]) >= MAX_ERR_ANG)
    {
        if (!servo2.is_set && condition)
        {
            std_msgs::Float32 msg;
            msg.data = joint_positions_home_[3];
            servo2_pub_.publish(msg);
            servo2.is_set = true;
        }
    }
    else servo2.is_on_pos = true;
}

void PrinterControl::servo2Update()
{
    servo2Update(true);
}

void PrinterControl::stepper1Update(bool condition)
{
    // stepper1 Lens_X_axis_trans
    if (abs(joint_positions_home_[2]-joint_positions_[2]) >= MAX_ERR_POS)
    {
        if (!stepper1.is_set && condition)
        {
            std_msgs::Float32 msg;
            msg.data = joint_positions_home_[2];
            stepper1_target_pub_.publish(msg);
            stepper1.is_set = true;
        }
    }
    else stepper1.is_on_pos = true;
}

void PrinterControl::stepper1Update()
{
    stepper1Update(true);
}

void PrinterControl::stepper2Update(bool condition)
{
    // stepper2 Lens_Y_axis_trans
    if (abs(joint_positions_home_[1]-joint_positions_[1]) >= MAX_ERR_POS)
    {
        if (!stepper2.is_set && condition)
        {
            std_msgs::Float32 msg;
            msg.data = joint_positions_home_[1];
            stepper2_target_pub_.publish(msg);
            stepper2.is_set = true;
        }
    }
    else stepper2.is_on_pos = true;
}

void PrinterControl::stepper2Update()
{
    stepper2Update(true);
}

void PrinterControl::linActuatorUpdate()
{
    // linear motor MainFrame_pitch
    if (!lin_actuator.is_on_pos &&
        lin_actuator_control(joint_positions_home_[0] - joint_positions_[0]))
    {
        lin_actuator.is_on_pos = true;
    }
}

void PrinterControl::resetActuatorsStruct()
{
    servo1.is_on_pos = false;
    servo1.is_set = false;
    servo2.is_on_pos = false;
    servo2.is_set = false;
    stepper1.is_on_pos = false;
    stepper1.is_set = false;
    stepper2.is_on_pos = false;
    stepper2.is_set = false;
    lin_actuator.is_on_pos = false;
}

bool PrinterControl::servosOnPos()
{
    return (servo2.is_on_pos && servo1.is_on_pos);
}

bool PrinterControl::steppersOnPos()
{
    return (stepper2.is_on_pos && stepper1.is_on_pos);
}

geometry_msgs::Quaternion PrinterControl::createQuaternionMsg(double roll_deg, double pitch_deg, double yaw_deg)
{
    tf2::Quaternion my_quaternion;
    my_quaternion.setRPY(roll_deg/180.0*M_PI, pitch_deg/180.0*M_PI, yaw_deg/180.0*M_PI);
    geometry_msgs::Quaternion my_quaternion_msg = tf2::toMsg(my_quaternion);
    return my_quaternion_msg;
}

bool PrinterControl::lin_actuator_control(double error)
{
    std_msgs::Float32 msg;

    if (error < MAX_ERR_ANG)
    {
        msg.data = 0;
        tilt_pub_.publish(msg);
        return true;
    }

    // P controller
    double u = KP_GAIN * (error);

    //saturation ?
//    if (u > 1000) u=1000;

    // Publish msg
    msg.data = (int8_t)u;
    tilt_pub_.publish(msg);

    return false;
}

void PrinterControl::targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ROS_INFO_STREAM("targetCmdCallback");
    if (printer_state_ == IDLE)
    {
        printing_pose_found_ = false;
        printing_point_ = *msg;
        go_print_ = true;
        ROS_INFO_STREAM("New printing_point: \nx: " << printing_point_.x << "\ny: " << printing_point_.y << "\nz: " << printing_point_.z) ;
    }
    else
    {
        ROS_WARN("Cannot update \"printing_point_\", printer is not in state IDLE");
    }

}

void PrinterControl::printerStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if (msg->data == PrinterState::HOME)
    {
        if (printer_state_ != HOME)
        {
            ROS_INFO_STREAM("printerStateCallback: HOME");
            go_home_ = true;
            joint_positions_target_ = joint_positions_home_;
            // reset timers
            // ...
        }
        else
        {
            ROS_INFO_STREAM("Printer already HOME");
        }

    }
    else if (msg->data == PrinterState::IDLE)
    {
        /*I do not want to receive IDLE if printer is BUSY*/
        if (printer_state_ != IDLE && printer_state_ != BUSY)
        {
            ROS_INFO_STREAM("printerStateCallback: IDLE");
            joint_positions_target_ = joint_positions_idle1_;
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

void PrinterControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::vector<double>::const_iterator first = msg->position.begin() + 8;
    std::vector<double> newVec(first, msg->position.end());
    joint_positions_ = newVec;
}