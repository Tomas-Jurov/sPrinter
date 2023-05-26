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
  , printer_state_(HOME)
  , go_home_(false), go_idle_(false), go_print_(false)
  , need_initialize_(true)
  , need_go_home_(false)
  , printing_pose_found_(false)
  , lin_actuator_last_time_(ros::Time::now())
  , counter_printing_point_(0), integrator_(0.0)
  {
    //Constructor
      joint_positions_rel_target_.resize(5);

}

void PrinterControl::update()
{
    if (go_home_) goHome();

    if (go_idle_) goIdle();

    if (go_print_) goPrint();

    /*check if need to go to home pos*/
    if (printer_state_ == IDLE && need_go_home_)
    {
        go_home_ = true;
        need_go_home_ = false;
        setAbsAndRelTargets(joint_positions_home_);
    }

    /*check if printing finished*/
    if (printer_state_ == PRINTING &&
            abs(printing_start_timestamp_.toSec()-ros::Time::now().toSec()) > PRINTING_TIMEOUT)
    {
        counter_printing_point_++;
        printer_state_ = IDLE;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
        idle_start_timestamp_ = ros::Time::now();
        ROS_INFO_STREAM("[Printer Control] printing of point [" << counter_printing_point_ << "] finished");
    }

    /*idle2*/
    if (printer_state_ == IDLE &&
            abs(idle_start_timestamp_.toSec()-ros::Time::now().toSec()) > IDLE_TIMEOUT)
    {
        setAbsAndRelTargets(joint_positions_idle2_);
        go_idle_ = true;
        if (!need_initialize_) need_initialize_ = true;
        ROS_INFO_STREAM("[Printer Control] idle position timeout - moving to IDLE2 position");
        ROS_INFO_STREAM("[Printer Control] need initialize cause printer_state_ = IDLE2");
    }

//    /*if robot printing, display remaining printing time*/
//    if(printer_state_ == PRINTING &&
//            (int)abs(printing_start_timestamp_.toSec()-ros::Time::now().toSec()) % 60 == 0)
//    {
//        ROS_INFO_STREAM("[Printer Control] remaining printing time: " << (int)abs(printing_start_timestamp_.toSec()-ros::Time::now().toSec()) / 60 << "min");
//    }

}

void PrinterControl::goPrint()
{
    lin_actuator_last_time_ = ros::Time::now();
    if (printer_state_ != BUSY) printer_state_ = BUSY;
    if (!printing_pose_found_)
    {
        geometry_msgs::PoseStamped tmp;
        tmp.pose.orientation = quaternion_world_sun;
        tmp.header.frame_id = PRINTING_FRAME;

        //transform orientation to sun from world to lens_focal_static_frame
        geometry_msgs::PoseStamped pose_static_sun = tf_buffer_.transform(
                tmp, PRINTING_FRAME);

        //set printing pose in lens_focal_static_frame
        printing_pose_.pose.orientation = pose_static_sun.pose.orientation;
        printing_pose_.pose.position = printing_point_;
        printing_pose_.header.frame_id = PRINTING_FRAME;
        printing_pose_.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped printing_pose_in_base_link_ = tf_buffer_.transform(
                printing_pose_, "base_link");

        ROS_INFO_STREAM("[Printer Control] desired_pose_in_base_link_ \npose\nx: " << printing_pose_in_base_link_.pose.position.x <<
                                                           "\ny: "<< printing_pose_in_base_link_.pose.position.y<<
                                                           "\nz: " << printing_pose_in_base_link_.pose.position.z <<
                                                           "\norientation\nw: "<< printing_pose_in_base_link_.pose.orientation.w<<
                                                           "\nx: "<< printing_pose_in_base_link_.pose.orientation.x<<
                                                           "\ny: "<< printing_pose_in_base_link_.pose.orientation.y<<
                                                           "\nz: "<< printing_pose_in_base_link_.pose.orientation.z);

        ik_srv_.request.pose = printing_pose_in_base_link_;
        ik_client_.call(ik_srv_);

        if (!ik_srv_.response.joint_states.size())
        {
            ROS_INFO_STREAM("[Printer Control] IK failure");
            printer_state_ = FAILURE;
            std_msgs::Int8 msg;
            msg.data = printer_state_;
            printer_state_pub_.publish(msg);
            setAbsAndRelTargets(joint_positions_); //important
        }

        setAbsAndRelTargets(ik_srv_.response.joint_states);
        printing_pose_found_ = true;
    }

    stepper2Update();
    stepper1Update();
    linActuatorUpdate();
    servo2Update(steppersOnPos());
    servo1Update(steppersOnPos());

    // on printing pos
    if (servosOnPos() && steppersOnPos() && lin_actuator.is_on_pos)
    {
        ROS_INFO_STREAM("[Printer Control] on printing position");
        printer_state_ = PRINTING;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
        go_print_ = false;
        resetActuatorsStruct();
        printing_start_timestamp_ = ros::Time::now();
        ROS_INFO_STREAM("[Printer Control] printing started");
    }
}

void PrinterControl::goIdle()
{
    lin_actuator_last_time_ = ros::Time::now();
    if (need_initialize_)
    {
        ROS_INFO_STREAM("[Printer Control] initializing");
        printer_state_ = INIT;
        gps_client_.call(gps_srv_);
        quaternion_world_sun = gps_srv_.response.orientation;
        
        try
        {
            if (sqrt(pow(quaternion_world_sun.x,2)+
                pow(quaternion_world_sun.y,2)+
                pow(quaternion_world_sun.z,2)+
                pow(quaternion_world_sun.w,2))
                == 1.0)
            {
                ROS_INFO_STREAM("[Printer Control] GPS node sent valid orientation to sun");
            }
            else
            {
                ROS_ERROR("[Printer Control] received non-valid orientation to sun");
                ROS_WARN("[Printer Control] setting default values for orientation to sun");
                quaternion_world_sun.x = 0;
                quaternion_world_sun.y = 0;
                quaternion_world_sun.z = 0;
                quaternion_world_sun.w = 1;
            }
        }
        catch(...)
        {
            ROS_ERROR("[Printer Control] err initializing [try-catch]");
        }

        need_initialize_ = false;
        printer_state_ = BUSY;
        ROS_INFO_STREAM("[Printer Control] initializing finished");
    }
    else if (printer_state_ != BUSY) printer_state_ = BUSY;

    if (need_go_home_)
    {
        servo2Update(steppersOnPos());
        servo1Update(steppersOnPos());
        linActuatorUpdate();
        stepper2Update();
        stepper1Update();
    }
    else
    {
        stepper2Update();
        stepper1Update();
        linActuatorUpdate();
        servo2Update(steppersOnPos());
        servo1Update(steppersOnPos());
    }


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
        if (isInIdle2())
        {
            printer_state_ = IDLE2;
            ROS_INFO_STREAM("[Printer Control] on IDLE2 position");
        }
        else ROS_INFO_STREAM("[Printer Control] on IDLE position");
    }

}

void PrinterControl::goHome()
{
    lin_actuator_last_time_ = ros::Time::now();
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
        ROS_INFO_STREAM("[Printer Control] on HOME position");
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
    if (abs(joint_positions_abs_target_[4]-joint_positions_[4]) >= ERR_TRESHOLD_ANG)
    {
        if (!servo1.is_set && condition)
        {
            ROS_INFO_STREAM("[Printer Control] target on servo1: \nabs:" << joint_positions_abs_target_[4]);
            std_msgs::Float32 msg;
            msg.data = joint_positions_abs_target_[4];
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
    if (abs(joint_positions_abs_target_[3]-joint_positions_[3]) >= ERR_TRESHOLD_ANG)
    {
        if (!servo2.is_set && condition)
        {
            ROS_INFO_STREAM("[Printer Control] target on servo2: \nabs:" << joint_positions_abs_target_[3]);
            std_msgs::Float32 msg;
            msg.data = joint_positions_abs_target_[3];
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
    if (abs(joint_positions_abs_target_[2]-joint_positions_[2]) >= ERR_TRESHOLD_POS)
    {
        if (!stepper1.is_set && condition)
        {
            ROS_INFO_STREAM("[Printer Control] target on stepper1: \nabs:" << joint_positions_abs_target_[2] <<
                                                                         "\nrel: " << joint_positions_rel_target_[2]);
            std_msgs::Float32 msg;
            msg.data = joint_positions_rel_target_[2];
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
    if (abs(joint_positions_abs_target_[1]-joint_positions_[1]) >= ERR_TRESHOLD_POS)
    {
        if (!stepper2.is_set && condition)
        {
            ROS_INFO_STREAM("[Printer Control] target on stepper2: \nabs:" << joint_positions_abs_target_[1] <<
                                                                           "\nrel: " << joint_positions_rel_target_[1]);
            std_msgs::Float32 msg;
            msg.data = joint_positions_rel_target_[1];
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
        lin_actuator_control(joint_positions_abs_target_[0] - joint_positions_[0]))
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

bool PrinterControl::isInIdle2()
{
    if (abs(joint_positions_idle2_[0]-joint_positions_[0]) < ERR_TRESHOLD_ANG &&
    abs(joint_positions_idle2_[1]-joint_positions_[1]) < ERR_TRESHOLD_POS && // stepper
    abs(joint_positions_idle2_[2]-joint_positions_[2]) < ERR_TRESHOLD_POS && // stepper
    abs(joint_positions_idle2_[3]-joint_positions_[3]) < ERR_TRESHOLD_ANG&&
    abs(joint_positions_idle2_[4]-joint_positions_[4]) <ERR_TRESHOLD_ANG) return true;


    return false;
}

bool PrinterControl::lin_actuator_control(double error)
{
    std_msgs::Int8 msg;

    if (error < ERR_TRESHOLD_ANG)
    {
        msg.data = 0;
        tilt_pub_.publish(msg);
        integrator_ = 0;
        return true;
    }

    // PI controller
    ros::Duration dt = ros::Time::now() - lin_actuator_last_time_;
    integrator_ += error * dt.toSec();
    double u = KP_GAIN * (error) + KI_GAIN*integrator_;

    // Publish msg
    msg.data = (int8_t)u;
    tilt_pub_.publish(msg);

    ROS_INFO_STREAM("[Printer Control] linear motor speed: "<< u);

    return false;
}

void PrinterControl::reset_goes()
{
    go_print_ = false;
    go_home_ = false;
    go_idle_ = false;
}

void PrinterControl::setAbsAndRelTargets(std::vector<double> joint_positions_abs_target)
{
    joint_positions_abs_target_ = joint_positions_abs_target;
    for (int i=0; i<joint_positions_home_.size(); i++)
    {
        joint_positions_rel_target_[i] = joint_positions_abs_target[i]-joint_positions_[i];
    }
}

void PrinterControl::targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ROS_INFO_STREAM("[Printer Control] targetCmdCallback");
    if (printer_state_ == IDLE || printer_state_ == IDLE2)
    {
        printing_pose_found_ = false;
        printing_point_ = *msg;
        go_print_ = true;
        ROS_INFO_STREAM("[Printer Control] new printing_point: \nx: " << printing_point_.x << "\ny: " << printing_point_.y << "\nz: " << printing_point_.z) ;
    }
    else
    {
        ROS_WARN("[Printer Control] cannot update \"printing_point_\", printer is not in state IDLE");
    }

}

void PrinterControl::printerStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if (msg->data == PrinterState::HOME)
    {
        if (printer_state_ != HOME)
        {
            ROS_INFO_STREAM("[Printer Control] printerStateCallback: HOME");
            resetActuatorsStruct();
            reset_goes();

            if (printer_state_ == IDLE)
            {
                go_home_ = true;
                setAbsAndRelTargets(joint_positions_home_);
            }
            else
            {
                go_idle_ = true;
                need_go_home_ = true;
                setAbsAndRelTargets(joint_positions_idle1_);
            }
        }
        else
        {
            ROS_WARN("[Printer Control] printerStateCallback: (already) HOME");
        }

    }
    else if (msg->data == IDLE)
    {
        /*I do not want to receive IDLE if printer is BUSY*/
        if (printer_state_ == HOME)
        {
            ROS_INFO_STREAM("[Printer Control] printerStateCallback: IDLE");
            setAbsAndRelTargets(joint_positions_idle1_);
            ROS_INFO_STREAM("OK");
            go_idle_ = true;
        }
        else
        {
            ROS_WARN("[Printer Control] printerStateCallback: (already) IDLE");
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