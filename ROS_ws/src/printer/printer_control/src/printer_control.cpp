#include "../include/printer_control.h"
#include "../include/printer_joint_positions.h"

PrinterControl::PrinterControl(const ros::Publisher& printer_state_pub, const ros::Publisher& tilt_pub,
                               const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                               const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                               const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                               const ros::Publisher& status_pub,
                               const ros::ServiceClient& gps_client, const ros::ServiceClient& ik_client)
  : printer_state_pub_(printer_state_pub)
  , tilt_pub_(tilt_pub)
  , stepper1_speed_pub_(stepper1_speed_pub)
  , stepper2_speed_pub_(stepper2_speed_pub)
  , stepper1_target_pub_(stepper1_target_pub)
  , stepper2_target_pub_(stepper2_target_pub)
  , servo1_pub_(servo1_pub)
  , servo2_pub_(servo2_pub)
  , status_pub_(status_pub)
  , gps_client_(gps_client)
  , ik_client_(ik_client)
  , tf_buffer_()
  , tf_listener_(tf_buffer_)
  , printer_state_(HOME)
  , go_home_(false), go_idle_(false), go_print_(false)
  , need_initialize_(true)
  , need_go_home_(false)
  , printing_pose_found_(false), printing_time_blocked_(false)
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
        lin_actuator_last_time_ = ros::Time::now();
        setAbsAndRelTargets(joint_positions_home_);
    }

    /*check if printing finished*/
    if (printer_state_ == PRINTING &&
            abs(printing_start_timestamp_.toSec()-ros::Time::now().toSec()) >= PRINTING_TIMEOUT)
    {
        counter_printing_point_++;
        printer_state_ = IDLE;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
        idle_start_timestamp_ = ros::Time::now();
        publishStatus(LOG_LEVEL_T::OK, "Printing of point [" + std::to_string(counter_printing_point_) + "] finished");
    }

    /*if robot printing, display remaining printing time*/
    displayPrintingTime();

    /*idle2*/
    if (printer_state_ == IDLE &&
            abs(idle_start_timestamp_.toSec()-ros::Time::now().toSec()) > IDLE_TIMEOUT)
    {
        setAbsAndRelTargets(joint_positions_idle2_);
        go_idle_ = true;
        lin_actuator_last_time_ = ros::Time::now();
        publishStatus(LOG_LEVEL_T::WARN, "Idle position timeout - moving to IDLE2 position");
    }
}

void PrinterControl::goPrint()
{
    lin_actuator_last_time_ = ros::Time::now();

    if (need_initialize_)
    {
        publishStatus(LOG_LEVEL_T::OK, "Initializing");
        printer_state_ = INIT;
        if (gps_client_.call(gps_srv_))
        {
            quaternion_world_sun = gps_srv_.response.orientation;
        }
        else
        {
            publishStatus(LOG_LEVEL_T::ERROR, "Non-valid orientation to sun");
            publishStatus(LOG_LEVEL_T::WARN, "Setting default values for orientation to sun");
            quaternion_world_sun.x = 0;
            quaternion_world_sun.y = 0;
            quaternion_world_sun.z = 0;
            quaternion_world_sun.w = 1;
        }

        need_initialize_ = false;
        printer_state_ = BUSY;
        publishStatus(LOG_LEVEL_T::OK, "Initializing finished");
    }
    else if (printer_state_ != BUSY) printer_state_ = BUSY;

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

        publishStatus(LOG_LEVEL_T::OK,
                      "Desired_pose_in_base_link_ \npose\nx: " + std::to_string(printing_pose_in_base_link_.pose.position.x) +
                                                                                                  "\ny: "+ std::to_string(printing_pose_in_base_link_.pose.position.y)+
                                                                                                  "\nz: " + std::to_string(printing_pose_in_base_link_.pose.position.z) +
                                                                                                  "\norientation\nw: "+ std::to_string(printing_pose_in_base_link_.pose.orientation.w)+
                                                                                                  "\nx: "+ std::to_string(printing_pose_in_base_link_.pose.orientation.x)+
                                                                                                  "\ny: "+ std::to_string(printing_pose_in_base_link_.pose.orientation.y)+
                                                                                                  "\nz: "+ std::to_string(printing_pose_in_base_link_.pose.orientation.z));

        ik_srv_.request.pose = printing_pose_in_base_link_;
        ik_client_.call(ik_srv_);

        if (!ik_srv_.response.joint_states.size())
        {
            publishStatus(LOG_LEVEL_T::ERROR, "IK failure");
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
        publishStatus(LOG_LEVEL_T::OK, "On printing position");
        printer_state_ = PRINTING;
        std_msgs::Int8 msg;
        msg.data = printer_state_;
        printer_state_pub_.publish(msg);
        go_print_ = false;
        resetActuatorsStruct();
        printing_start_timestamp_ = ros::Time::now();
        publishStatus(LOG_LEVEL_T::OK, "Printing started");
    }
}

void PrinterControl::goIdle()
{
    lin_actuator_last_time_ = ros::Time::now();
    if (printer_state_ != BUSY) printer_state_ = BUSY;

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
            publishStatus(LOG_LEVEL_T::OK, "On IDLE2 position");
        }
        else publishStatus(LOG_LEVEL_T::OK, "On IDLE position");
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
        publishStatus(LOG_LEVEL_T::OK, "On HOME position");
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
            publishStatus(LOG_LEVEL_T::OK, "Target on servo1: \nabs:" + std::to_string(joint_positions_abs_target_[4]));
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
            publishStatus(LOG_LEVEL_T::OK, "Target on servo2: \nabs:" + std::to_string(joint_positions_abs_target_[3]));
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
            publishStatus(LOG_LEVEL_T::OK, "Target on stepper1: \nabs:" + std::to_string(joint_positions_abs_target_[2]) +
                                           "\nrel: " + std::to_string(joint_positions_rel_target_[2]));
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
            publishStatus(LOG_LEVEL_T::OK, "Target on stepper2: \nabs:" + std::to_string(joint_positions_abs_target_[1]) +
                                                                                          "\nrel: " + std::to_string(joint_positions_rel_target_[1]));
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
            linActuatorControl(joint_positions_abs_target_[0] - joint_positions_[0]))
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

bool PrinterControl::linActuatorControl(double error)
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

    publishStatus(LOG_LEVEL_T::OK, "Linear motor speed: "+ std::to_string(u)); //std::to_string

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

void PrinterControl::displayPrintingTime()
{
    if(printer_state_ == PRINTING &&
       ((int)abs(printing_start_timestamp_.toSec()-ros::Time::now().toSec())) % 60 == 0 /* &&
       (PRINTING_TIMEOUT - (int)abs(printing_start_timestamp_.toSec()-ros::Time::now().toSec())) / 60 != 0*/)
    {
        if (!printing_time_blocked_)
        {
            publishStatus(LOG_LEVEL_T::OK, "Remaining printing time: "
            + std::to_string((PRINTING_TIMEOUT - (int)abs(printing_start_timestamp_.toSec()-ros::Time::now().toSec())) / 60) + "min");
            printing_time_blocked_ = true;
        }
    }
    else if (printing_time_blocked_)
    {
        printing_time_blocked_ = false;
    }
}

void PrinterControl::publishStatus(const int8_t logger_level, const std::string& message)
{
    status_msg_.level = logger_level;
    status_msg_.name = "Printer Control";
    status_msg_.message = message;

    if (!status_msg_.message.empty())
        status_pub_.publish(status_msg_);
}

void PrinterControl::targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    publishStatus(LOG_LEVEL_T::OK, "TargetCmdCallback");
    if (printer_state_ == IDLE || printer_state_ == IDLE2)
    {
        printing_pose_found_ = false;
        printing_point_ = *msg;
        go_print_ = true;
        lin_actuator_last_time_ = ros::Time::now();
        if (printer_state_ == IDLE2) need_initialize_ = true;
        publishStatus(LOG_LEVEL_T::OK, "New printing_point: \nx: " +std::to_string(printing_point_.x) + "\ny: " + std::to_string(printing_point_.y) + "\nz: "  +std::to_string(printing_point_.z));


    }
    else
    {
        publishStatus(LOG_LEVEL_T::WARN, "Cannot update \"printing_point_\", printer is not in state IDLE");
    }

}

void PrinterControl::printerStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if (msg->data == PrinterState::HOME)
    {
        if (printer_state_ != HOME)
        {
            publishStatus(LOG_LEVEL_T::OK, "PrinterStateCallback: HOME");
            resetActuatorsStruct();
            reset_goes();

            if (printer_state_ == IDLE)
            {
                go_home_ = true;
                lin_actuator_last_time_ = ros::Time::now();
                setAbsAndRelTargets(joint_positions_home_);
            }
            else
            {
                go_idle_ = true;
                need_go_home_ = true;
                lin_actuator_last_time_ = ros::Time::now();
                setAbsAndRelTargets(joint_positions_idle1_);
            }
        }
        else
        {
            publishStatus(LOG_LEVEL_T::WARN, "PinterStateCallback: (already) HOME");
        }

    }
    else if (msg->data == IDLE)
    {
        /*I do not want to receive IDLE if printer is BUSY*/
        if (printer_state_ == HOME)
        {
            publishStatus(LOG_LEVEL_T::OK, "PrinterStateCallback: IDLE");
            setAbsAndRelTargets(joint_positions_idle1_);
            go_idle_ = true;
            lin_actuator_last_time_ = ros::Time::now();
        }
        else
        {
            publishStatus(LOG_LEVEL_T::WARN, "PrinterStateCallback: (already) IDLE");
        }
    }
}

void PrinterControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::vector<double>::const_iterator first = msg->position.begin() + 8;
    std::vector<double> newVec(first, msg->position.end());
    joint_positions_ = newVec;
}