#include "../include/printer_control.h"
#include "../include/printer_joint_positions.h"

PrinterControl::PrinterControl(const ros::Publisher& printer_state_pub, const ros::Publisher& tilt_pub,
                               const ros::Publisher& stepper1_speed_pub, const ros::Publisher& stepper2_speed_pub,
                               const ros::Publisher& stepper1_target_pub, const ros::Publisher& stepper2_target_pub,
                               const ros::Publisher& servo1_pub, const ros::Publisher& servo2_pub,
                               const ros::Publisher& status_pub, const ros::ServiceClient& gps_client,
                               const ros::ServiceClient& ik_client)
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
  , go_home_(false)
  , go_idle_(false)
  , go_print_(false)
  , need_initialize_(true)
  , need_go_home_(false)
  , ik_pose_found_(false)
  , printing_time_blocked_(false)
  , lin_actuator_last_time_(ros::Time::now())
  , counter_printing_point_(0)
{
  // Constructor
  joint_positions_rel_target_.resize(5);
}

/// \brief Called in every spin, updates printer control node.
void PrinterControl::update()
{
  if (go_home_)
    goHome();

  if (go_idle_)
    goIdle();

  if (go_print_)
    goPrint();

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
      abs(printing_start_timestamp_.toSec() - ros::Time::now().toSec()) >= PRINTING_TIMEOUT)
  {
    counter_printing_point_++;
    printer_state_ = IDLE;
    std_msgs::Int8 msg;
    msg.data = printer_state_;
    printer_state_pub_.publish(msg);
    idle_start_timestamp_ = ros::Time::now();
    publishStatus(LOG_LEVEL_T::OK, "Printing of point [" + std::to_string(counter_printing_point_) + "] finished");
    ROS_DEBUG_STREAM("Printing of point [" + std::to_string(counter_printing_point_) + "] finished");
  }

  /*if robot printing, display remaining printing time*/
  displayPrintingTime();

  /*idle2*/
  if ((printer_state_ == IDLE || printer_state_ == FAILURE) &&
      abs(idle_start_timestamp_.toSec() - ros::Time::now().toSec()) > IDLE_TIMEOUT)
  {
    setAbsAndRelTargets(computeIdle2JointPositions());
    go_idle_ = true;
    ROS_DEBUG_STREAM("Rotating lens away from the sun");
    publishStatus(LOG_LEVEL_T::OK, "Rotating lens away from the sun");
  }
}

/// \brief Called when variable go_print_ is set.
/// If needed, initializing (gps_serv_) is executed.
/// Then service for Inverse Kinematics is called and if succeed,
/// the response is used as new joint states absolute target. Updates actuators.
/// Afterwards, if actuator are on desired position, IDLE state is returned.
/// If IK didn't find the solution, there is a FAILURE state set and robot does not move.
void PrinterControl::goPrint()
{
  lin_actuator_last_time_ = ros::Time::now();

  if (need_initialize_)
  {
    ROS_DEBUG_STREAM("Initializing");
    printer_state_ = INIT;
    if (gps_client_.call(gps_srv_))
    {
      quaternion_world_sun = gps_srv_.response.orientation;
    }
    else
    {
      ROS_DEBUG_STREAM("Non-valid orientation to sun \n \"Setting default values for orientation to sun\"");

      quaternion_world_sun.x = 0;
      quaternion_world_sun.y = 0;
      quaternion_world_sun.z = 0;
      quaternion_world_sun.w = 1;
    }

    need_initialize_ = false;
    printer_state_ = BUSY;
    ROS_DEBUG_STREAM("Initializing finished");
  }
  else if (printer_state_ != BUSY)
    printer_state_ = BUSY;

  if (!ik_pose_found_)
  {
    geometry_msgs::PoseStamped tmp;
    tmp.pose.orientation = quaternion_world_sun;
    tmp.header.frame_id = PRINTING_FRAME;

    // transform orientation to sun from world to ref_print_frame
    geometry_msgs::PoseStamped pose_static_sun = tf_buffer_.transform(tmp, PRINTING_FRAME);

    // set printing pose in ref_print_frame
    printing_pose_.pose.orientation = pose_static_sun.pose.orientation;
    printing_pose_.pose.position = printing_point_;
    printing_pose_.header.frame_id = PRINTING_FRAME;
    printing_pose_.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped printing_pose_in_base_link_ = tf_buffer_.transform(printing_pose_, "base_link");

    ROS_DEBUG_STREAM("Printing_pose_in_base_link_ \npose\nx: " +
                     std::to_string(printing_pose_in_base_link_.pose.position.x) + "\ny: " +
                     std::to_string(printing_pose_in_base_link_.pose.position.y) + "\nz: " +
                     std::to_string(printing_pose_in_base_link_.pose.position.z) + "\norientation\nw: " +
                     std::to_string(printing_pose_in_base_link_.pose.orientation.w) + "\nx: " +
                     std::to_string(printing_pose_in_base_link_.pose.orientation.x) + "\ny: " +
                     std::to_string(printing_pose_in_base_link_.pose.orientation.y) + "\nz: " +
                     std::to_string(printing_pose_in_base_link_.pose.orientation.z));

    ik_srv_.request.pose = printing_pose_in_base_link_;

    if (ik_client_.call(ik_srv_) && !ik_srv_.response.joint_states.size())
    {
      ROS_DEBUG_STREAM("State changed to FAILURE");
      printer_state_ = FAILURE;
      std_msgs::Int8 msg;
      msg.data = printer_state_;
      printer_state_pub_.publish(msg);
      go_print_ = false;
      resetActuatorsStruct();
    }
    else
    {
      publishStatus(LOG_LEVEL_T::OK, "Moving to printing position");
      ROS_DEBUG_STREAM("Moving to printing position");
      setAbsAndRelTargets(ik_srv_.response.joint_states);
      ik_pose_found_ = true;
    }
  }

  if (ik_pose_found_)
  {
    stepper2Update();
    stepper1Update();
    linActuatorUpdate();
    servo2Update(steppersOnPos());
    servo1Update(steppersOnPos());
  }

  // on printing pos
  if (servosOnPos() && steppersOnPos() && lin_actuator.is_on_pos)
  {
    publishStatus(LOG_LEVEL_T::OK, "Printer is on PRINTING position");
    ROS_DEBUG_STREAM("Printer is on printing position");
    printer_state_ = PRINTING;
    std_msgs::Int8 msg;
    msg.data = printer_state_;
    printer_state_pub_.publish(msg);
    go_print_ = false;
    resetActuatorsStruct();
    printing_start_timestamp_ = ros::Time::now();
    ROS_DEBUG_STREAM("Printing started");
  }
}

/// \brief Called when variable go_idle_ is set. Updates actuators.
/// There are 2 orders for udpating actuator - when going from home/to home.
/// If actuators are on desired position, IDLE state is returned.
void PrinterControl::goIdle()
{
  lin_actuator_last_time_ = ros::Time::now();

  if (printer_state_ != BUSY_TO_IDLE && printer_state_ == HOME)
    printer_state_ = BUSY_TO_IDLE;
  if (printer_state_ != BUSY && printer_state_ != BUSY_TO_IDLE)
    printer_state_ = BUSY;

  if (need_go_home_)
  {
    servo2Update();
    servo1Update();
    linActuatorUpdate();
    stepper2Update(servosOnPos());
    stepper1Update(servosOnPos());
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
      ROS_DEBUG_STREAM("Printer is on IDLE2 position");
      publishStatus(LOG_LEVEL_T::OK, "Printer is on IDLE position");
      if (!need_go_home_)
        printer_state_ = IDLE2;
    }
    else if (!need_go_home_)
    {
      ROS_DEBUG_STREAM("Printer is on IDLE position");
      publishStatus(LOG_LEVEL_T::OK, "Printer is on IDLE position");
    }
    else
    {
      ROS_DEBUG_STREAM("Printer is on IDLE position");
    }
  }
}

/// \brief Called when variable go_home_ is set. Updates actuators.
/// If actuators are on desired position, IDLE state is returned.
void PrinterControl::goHome()
{
  lin_actuator_last_time_ = ros::Time::now();
  if (!need_initialize_)
    need_initialize_ = true;
  if (printer_state_ != BUSY)
    printer_state_ = BUSY;
  servo2Update();
  servo1Update();
  linActuatorUpdate();
  stepper2Update(servosOnPos());
  stepper1Update(servosOnPos());

  // on home pos
  if (servosOnPos() && steppersOnPos() && lin_actuator.is_on_pos)
  {
    publishStatus(LOG_LEVEL_T::OK, "Printer is on HOME position");
    ROS_DEBUG_STREAM("Printer is on HOME position");
    printer_state_ = HOME;
    std_msgs::Int8 msg;
    msg.data = printer_state_;
    printer_state_pub_.publish(msg);
    go_home_ = false;
    resetActuatorsStruct();
  }
}

/// \brief Set absolute target to servo1 if not already done.
/// \param condition Affects setting target to servo, behaves as blocker if false.
void PrinterControl::servo1Update(bool condition)
{
  // servo1 Lens_X_axis_rot
  if (abs(joint_positions_abs_target_[4] - joint_positions_[4]) >= ERR_TRESHOLD_ANG)
  {
    if (!servo1.is_set && condition)
    {
      ROS_DEBUG_STREAM("Target on servo1: \nabs:" + std::to_string(joint_positions_abs_target_[4]));
      std_msgs::Float32 msg;
      msg.data = joint_positions_abs_target_[4];
      servo1_pub_.publish(msg);
      servo1.is_set = true;
    }
  }
  else
    servo1.is_on_pos = true;
}

/// Override servo1Update(bool condition)
void PrinterControl::servo1Update()
{
  servo1Update(true);
}

/// \brief Set absolute target to servo2 if not already done.
/// \param condition Affects setting target to servo, behaves as blocker if false.
void PrinterControl::servo2Update(bool condition)
{
  // servo2 Lens_Y_axis_rot
  if (abs(joint_positions_abs_target_[3] - joint_positions_[3]) >= ERR_TRESHOLD_ANG)
  {
    if (!servo2.is_set && condition)
    {
      ROS_DEBUG_STREAM("Target on servo2: \nabs:" + std::to_string(joint_positions_abs_target_[3]));
      std_msgs::Float32 msg;
      msg.data = joint_positions_abs_target_[3];
      servo2_pub_.publish(msg);
      servo2.is_set = true;
    }
  }
  else
    servo2.is_on_pos = true;
}

/// Override servo2Update(bool condition)
void PrinterControl::servo2Update()
{
  servo2Update(true);
}

/// \brief Set relative target to stepper1 if not already done.
/// \param condition Affects setting target to stepper, behaves as blocker if false.
void PrinterControl::stepper1Update(bool condition)
{
  // stepper1 Lens_X_axis_trans
  if (abs(joint_positions_abs_target_[2] - joint_positions_[2]) >= ERR_TRESHOLD_POS)
  {
    if (!stepper1.is_set && condition)
    {
      ROS_DEBUG_STREAM("Target on stepper1: \nabs:" + std::to_string(joint_positions_abs_target_[2]) + "\nrel: " +
                       std::to_string(joint_positions_rel_target_[2]));
      std_msgs::Float32 msg;
      msg.data = joint_positions_rel_target_[2];
      stepper1_target_pub_.publish(msg);
      stepper1.is_set = true;
    }
  }
  else
    stepper1.is_on_pos = true;
}

/// Override stepper1Update(bool condition)
void PrinterControl::stepper1Update()
{
  stepper1Update(true);
}

/// \brief Set relative target to stepper2 if not already done.
/// \param condition Affects setting target to stepper, behaves as blocker if false.
void PrinterControl::stepper2Update(bool condition)
{
  // stepper2 Lens_Y_axis_trans
  if (abs(joint_positions_abs_target_[1] - joint_positions_[1]) >= ERR_TRESHOLD_POS)
  {
    if (!stepper2.is_set && condition)
    {
      ROS_DEBUG_STREAM("Target on stepper2: \nabs:" + std::to_string(joint_positions_abs_target_[1]) + "\nrel: " +
                       std::to_string(joint_positions_rel_target_[1]));
      std_msgs::Float32 msg;
      msg.data = joint_positions_rel_target_[1];
      stepper2_target_pub_.publish(msg);
      stepper2.is_set = true;
    }
  }
  else
    stepper2.is_on_pos = true;
}

/// Override stepper2Update(bool condition)
void PrinterControl::stepper2Update()
{
  stepper2Update(true);
}

/// \brief Set error to linActuatorControl(error).
void PrinterControl::linActuatorUpdate()
{
  // linear motor MainFrame_pitch
  if (!lin_actuator.is_on_pos && linActuatorControl(joint_positions_abs_target_[0] - joint_positions_[0]))
  {
    lin_actuator.is_on_pos = true;
  }
}

/// \brief Resets structs for actuators actuator.is_on_pos = false, actuator.is_set = false.
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

/// \brief
/// \return True if both servos are on desired position, false otherwise.
bool PrinterControl::servosOnPos()
{
  return (servo2.is_on_pos && servo1.is_on_pos);
}

/// \brief
/// \return True if both steppers are on desired position, false otherwise.
bool PrinterControl::steppersOnPos()
{
  return (stepper2.is_on_pos && stepper1.is_on_pos);
}

/// \brief
/// \return True if printer is on IDLE2 position (servo1 is rotated away from the sun), false otherwise.
bool PrinterControl::isInIdle2()
{
  if (abs(joint_positions_idle2_[4] - joint_positions_[4]) < ERR_TRESHOLD_ANG)
    return true;

  return false;
}
/// \brief Continuously updates linear motor's speeds.
/// \param error difference between target and actual joint state
/// \return True if abs(error) is under treshold, false otherwise.
bool PrinterControl::linActuatorControl(double error)
{
  std_msgs::Int8 msg;
  bool returnVal = false;
  double u;
  static double integrator(0.0);

  if (std::abs(error) < ERR_TRESHOLD_ANG)
  {
    u = 0;
    integrator = 0;
    returnVal = true;
  }
  else
  {
    // PI controller
    ros::Duration dt = ros::Time::now() - lin_actuator_last_time_;
    integrator += error * dt.toSec();
    u = KP_GAIN * (error) + KI_GAIN * integrator + K_DIR * (error / std::abs(error));
  }

  // Publish msg
  msg.data = static_cast<int8_t>(u);
  tilt_pub_.publish(msg);
  return returnVal;
}

/// \brief Reset go_print_, go_home_, go_idle_.
void PrinterControl::reset_goes()
{
  go_print_ = false;
  go_home_ = false;
  go_idle_ = false;
}

/// \brief Set absolute and relative joint states target.
/// Relative target is used only for steppers since we control them by adding the distance to the actual position.
/// \param joint_positions_abs_target
void PrinterControl::setAbsAndRelTargets(std::vector<double> joint_positions_abs_target)
{
  joint_positions_abs_target_ = joint_positions_abs_target;
  for (int i = 0; i < joint_positions_home_.size(); i++)
  {
    joint_positions_rel_target_[i] = joint_positions_abs_target[i] - joint_positions_[i];
  }
}

/// \brief Set idle2 joint states position.
/// Servo1 (joint_positions[4]) is rotated away from the sun, others joint positions remains the same.
/// \return idle2 joint states position
std::vector<double> PrinterControl::computeIdle2JointPositions()
{
  std::vector<double> joint_positions = joint_positions_;
  joint_positions[4] = joint_positions_idle2_[4];
  return joint_positions;
}

/// \brief Displays remaining printing time if printing.
/// Interval is set to display time every minute and if time under 1min, display also in 10s intervals
/// (eg 3min, 2min, 1min, 50s, 40s ...)
void PrinterControl::displayPrintingTime()
{
  int diff_s = ((int)abs(printing_start_timestamp_.toSec() - ros::Time::now().toSec()));

  if (printer_state_ == PRINTING && diff_s % 60 == 0)  // every minute
  {
    if (!printing_time_blocked_)
    {
      publishStatus(LOG_LEVEL_T::OK,
                    "Remaining printing time: " + std::to_string((PRINTING_TIMEOUT - diff_s) / 60) + "min");
      ROS_DEBUG_STREAM("Remaining printing time: " + std::to_string((PRINTING_TIMEOUT - diff_s) / 60) + "min");
      printing_time_blocked_ = true;
    }
  }
  else if (printer_state_ == PRINTING && (diff_s < 60 && diff_s % 10 == 0))  // every 10s if under minute
  {
    if (!printing_time_blocked_)
    {
      publishStatus(LOG_LEVEL_T::OK, "Remaining printing time: " + std::to_string(PRINTING_TIMEOUT - diff_s) + "s");
      ROS_DEBUG_STREAM("Remaining printing time: " + std::to_string(PRINTING_TIMEOUT - diff_s) + "s");
      printing_time_blocked_ = true;
    }
  }
  else if (printing_time_blocked_)
  {
    printing_time_blocked_ = false;
  }
}

/// \brief Publish status to status topic.
/// \param logger_level Level as OK, WARN, ERROR
/// \param message
void PrinterControl::publishStatus(const int8_t logger_level, const std::string& message)
{
  status_msg_.level = logger_level;
  status_msg_.name = "Printer Control";
  status_msg_.message = message;

  if (!status_msg_.message.empty())
    status_pub_.publish(status_msg_);
}

/// \brief Processes new printing point if received in state IDLE
/// \param msg
void PrinterControl::targetCmdCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("TargetCmdCallback");
  if (printer_state_ == IDLE || printer_state_ == IDLE2)
  {
    publishStatus(LOG_LEVEL_T::OK, "Received new printing request");
    ik_pose_found_ = false;
    printing_point_ = *msg;
    go_print_ = true;
    lin_actuator_last_time_ = ros::Time::now();
    resetActuatorsStruct();
    if (printer_state_ == IDLE2)
      need_initialize_ = true;
    ROS_DEBUG_STREAM("New printing_point: \nx: " + std::to_string(printing_point_.x) + "\ny: " +
                     std::to_string(printing_point_.y) + "\nz: " + std::to_string(printing_point_.z));
  }
  else
  {
    ROS_DEBUG_STREAM("Cannot update \"printing_point_\", printer is not in state IDLE");
  }
}

/// \brief Processes printer state change request from Task Manager node
/// \param msg
void PrinterControl::printerStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
  if (msg->data == PrinterState::HOME)
  {
    if (printer_state_ != HOME)
    {
      publishStatus(LOG_LEVEL_T::OK, "Printer moving to HOME");
      ROS_DEBUG_STREAM("PrinterStateCallback: HOME");
      resetActuatorsStruct();
      reset_goes();

      if (printer_state_ == IDLE || printer_state_ == BUSY_TO_IDLE)
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
      ROS_DEBUG_STREAM("PrinterStateCallback: (already) HOME");
    }
  }
  else if (msg->data == IDLE)
  {
    /*I do not want to receive IDLE if printer is BUSY*/
    if (printer_state_ == HOME)
    {
      publishStatus(LOG_LEVEL_T::OK, "Printer moving to IDLE");
      ROS_DEBUG_STREAM("PrinterStateCallback: IDLE");
      setAbsAndRelTargets(joint_positions_idle1_);
      go_idle_ = true;
      lin_actuator_last_time_ = ros::Time::now();
    }
    else
    {
      ROS_DEBUG_STREAM("PrinterStateCallback: printer state is not HOME, "
                       "cannot update to state IDLE");
    }
  }
}

/// \brief Processes joint states, set them to member variable.
/// \param msg
void PrinterControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::vector<double>::const_iterator first = msg->position.begin() + 8;
  std::vector<double> newVec(first, msg->position.end());
  joint_positions_ = newVec;
}