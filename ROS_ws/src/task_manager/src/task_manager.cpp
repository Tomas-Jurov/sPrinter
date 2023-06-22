#include "../include/task_manager.h"

namespace TaskManagerNS
{
TaskManager::TaskManager(const ros::Publisher& pose_cmd_pub, const ros::Publisher& printer_cmd_pub,
                         const ros::Publisher& printer_state_pub, ros::Publisher& pose_stop_pub,
                         const ros::Publisher& gps_do_pub, const ros::Publisher& gps_reset_pub,
                         const ros::Publisher& status_pub, const ros::Publisher& reset_odom_pub)
  : connected_(true)
  , state_(TaskManagerNS::START)
  , watchdog_last_(ros::Time::now())
  , pose_cmd_pub_(pose_cmd_pub)
  , printer_cmd_pub_(printer_cmd_pub)
  , printer_state_pub_(printer_state_pub)
  , pose_stop_pub_(pose_stop_pub)
  , gps_do_pub_(gps_do_pub)
  , gps_reset_pub_(gps_reset_pub)
  , status_pub_(status_pub)
  , reset_odom_pub_(reset_odom_pub)
  , pose_task_ptr_(nullptr)
{
}

void TaskManager::update()
{
  if (!watchdogCheck())
    return;

  if (state_ == INITIALIZING)
  {
  }
  else if (state_ == ROBOT_READY)
  {
    if (!printer_task_.empty())
    {
      publishPrinterTargetState(PrinterState::IDLE);
      setState(PRINTER_BUSY);
    }
    else if (pose_task_ptr_.get() != nullptr)
    {
      publishPoseTarget();
      setState(ROBOT_MOVING);
    }
  }
  else if (state_ == PRINTER_IDLE)
  {
    if (!printer_task_.empty())
    {
      publishPrinterTargetCmd();
      setState(PRINTER_BUSY);
    }
    else if (pose_task_ptr_.get() != nullptr)
    {
      publishPrinterTargetState(PrinterState::HOME);
      setState(PRINTER_BUSY);
    }
  }
}

bool TaskManager::watchdogCheck()
{
  if (ros::Duration(ros::Time::now() - watchdog_last_).toSec() > WATCHDOG_TIMEOUT)
  {
    if (connected_)
    {
      connected_ = false;
      publishStatus(LOG_LEVEL_T::ERROR, "Connection lost. Canceling all jobs.");
      ROS_ERROR("Connection lost. Canceling all jobs.");
      safetyStop();
    }
    return false;
  }
  else
  {
    if (!connected_)
    {
      ROS_WARN("Connection restored.");
      connected_ = true;
    }
    return true;
  }
}

void TaskManager::safetyStop()
{
  clearPoseTask();
  clearPrinterTask();

  if ((state_ != ROBOT_READY) && (state_ != START))
  {
    if (state_ == ROBOT_MOVING)
    {
      publishPoseStop();
    }
    else if (state_ == PRINTER_BUSY || state_ == PRINTER_IDLE)
    {
      publishPrinterTargetState(PrinterState::HOME);
    }
    setState(STOPPING);
  }
}

void TaskManager::reset()
{
  reset_odom_pub_.publish(std_msgs::Empty());
  gps_reset_pub_.publish(std_msgs::Empty());
  publishStatus(LOG_LEVEL_T::OK, "Reset successfull.");
  setState(START);
}

void TaskManager::heartbeatCallback(const std_msgs::Empty::ConstPtr& msg)
{
  watchdog_last_ = ros::Time::now();
}

bool TaskManager::safetyStopCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  publishStatus(LOG_LEVEL_T::WARN, "SAFETY STOP received! Caneling all jobs.");
  safetyStop();
  return true;
}

bool TaskManager::resetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  publishStatus(LOG_LEVEL_T::WARN, "RESET received! Caneling all jobs.");
  safetyStop();
  if (state_ == ROBOT_READY)
  {
    reset();
  }
  else
  {
    setState(RESETTING);
  }
  return true;
}

bool TaskManager::initializeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (state_ == START)
  {
    publishStatus(LOG_LEVEL_T::OK, "Starting a procedure to estimate global orientation.");
    publishStatus(LOG_LEVEL_T::WARN, "Robot will start to move soon!");
    gps_do_pub_.publish(std_msgs::Empty());
    setState(INITIALIZING);
  }
  else
    publishStatus(LOG_LEVEL_T::WARN, "Robot has been initialized before. Ignoring.");

  return true;
}

bool TaskManager::poseTaskCallback(sprinter_srvs::SetPose2D::Request& req, sprinter_srvs::SetPose2D::Response& res)
{
  if (state_ != ROBOT_MOVING)
  {
    pose_task_ptr_ = std::make_unique<geometry_msgs::Pose2D>(req.pose);
    publishStatus(LOG_LEVEL_T::OK, "Pose task set successfully.");
  }
  else
  {
    publishStatus(LOG_LEVEL_T::WARN, "The previous pose task hasn't been completed yet. Ignoring.");
  }

  return true;
}
bool TaskManager::printerTaskCallback(sprinter_srvs::SetPointArr::Request& req,
                                      sprinter_srvs::SetPointArr::Response& res)
{
  if (req.points.size() > 0)
  {
    if (state_ != PRINTER_BUSY)
    {
      for (auto& point : req.points)
      {
        printer_task_.emplace(point);
      }
      publishStatus(LOG_LEVEL_T::OK, "Received " + std::to_string(req.points.size()) + " printing points. Printer task "
                                                                                       "set successfully.");
    }
    else
    {
      publishStatus(LOG_LEVEL_T::WARN, "The previous printer task hasn't been completed yet. Ignoring.");
    }
  }
  else
  {
    publishStatus(LOG_LEVEL_T::WARN, "Got an empty printer task. Ignoring.");
  }

  return true;
}

void TaskManager::gpsProcessingCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (state_ == INITIALIZING && msg->data)
  {
    publishStatus(LOG_LEVEL_T::OK, "Estimation of global orientation finished.");
    setState(ROBOT_READY);
  }
}

void TaskManager::poseControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data && (state_ == ROBOT_MOVING || state_ == STOPPING))
  {
    if (state_ == ROBOT_MOVING)
    {
      publishStatus(LOG_LEVEL_T::OK, "Pose task completed successfully.");
    }
    else if (state_ == STOPPING)
    {
      publishStatus(LOG_LEVEL_T::OK, "Vehicle is safe now.");
    }
    setState(ROBOT_READY);
  }
  else if (msg->data && (state_ == RESETTING))
  {
    reset();
  }
}

void TaskManager::printerControlCallback(const std_msgs::Int8::ConstPtr& msg)
{
  if (state_ == PRINTER_BUSY)
  {
    if (msg->data == PrinterState::HOME)
    {
      if (pose_task_ptr_.get() != nullptr)
      {
        setState(ROBOT_READY);
      }
    }
    else if (msg->data == PrinterState::IDLE)
    {
      if (printer_task_.empty())
      {
        publishStatus(LOG_LEVEL_T::OK, "Printer task completed successfully.");
        setState(PRINTER_IDLE);
      }
      else
      {
        publishPrinterTargetCmd();
      }
    }
    else if (msg->data == PrinterState::FAILURE)
    {
      publishStatus(LOG_LEVEL_T::ERROR, "Printer Control unable to reach target point. Canceling task.");
      clearPrinterTask();
      setState(PRINTER_IDLE);
    }
    else
      publishStatus(LOG_LEVEL_T::WARN, "Unknown Printer Control state obrained. Ignoring.");
  }
  else if (msg->data == HOME)
  {
    if (state_ == STOPPING)
    {
      publishStatus(LOG_LEVEL_T::OK, "Vehicle is safe now.");
      setState(ROBOT_READY);
    }
    else if (state_ == RESETTING)
    {
      reset();
    }
  }
}

void TaskManager::setState(State state)
{
  if (state == state_)
    return;
  state_ = state;

  std::string state_str;
  switch (state)
  {
    case START:
      state_str = "START";
      break;
    case INITIALIZING:
      state_str = "INITIALIZING";
      break;
    case ROBOT_READY:
      state_str = "ROBOT READY";
      break;
    case ROBOT_MOVING:
      state_str = "ROBOT MOVING";
      break;
    case PRINTER_BUSY:
      state_str = "PRINTER BUSY";
      break;
    case PRINTER_IDLE:
      state_str = "PRINTER IDLE";
      break;
    case STOPPING:
      state_str = "STOPPING";
      break;
    case RESETTING:
      state_str = "RESETTING";
    default:
      state_str = "UNDEFINED";
      break;
  }

  publishStatus(LOG_LEVEL_T::OK, "Switching to " + state_str + " state.");
}

void TaskManager::publishStatus(const int8_t logger_level, const std::string& message)
{
  status_msg_.level = logger_level;
  status_msg_.name = "Task Manager";
  status_msg_.message = message;

  if (!status_msg_.message.empty())
    status_pub_.publish(status_msg_);
}

void TaskManager::publishPoseTarget()
{
  if (pose_task_ptr_.get() != nullptr)
  {
    publishStatus(LOG_LEVEL_T::OK, "Sending command to Pose Control.");
    publishStatus(LOG_LEVEL_T::WARN, "Robot will start to move soon!");
    pose_cmd_pub_.publish(*pose_task_ptr_.release());
  }
}

void TaskManager::publishPrinterTargetCmd()
{
  if (!printer_task_.empty())
  {
    publishStatus(LOG_LEVEL_T::OK, "Sending command to Printer Control.");
    publishStatus(LOG_LEVEL_T::WARN, "Lens will start to move soon!");
    printer_cmd_pub_.publish(printer_task_.front());
    printer_task_.pop();
  }
}

void TaskManager::publishPrinterTargetState(const PrinterState state)
{
  std::string state_str;
  switch (state)
  {
    case PrinterState::HOME:
      state_str = "HOME";
      break;
    case PrinterState::IDLE:
      state_str = "IDLE";
      break;
    case PrinterState::FAILURE:
      state_str = "FAILURE";
      break;
    default:
      state_str = "UNDEFINED";
      break;
  }
  publishStatus(LOG_LEVEL_T::OK, "Printer request state: " + state_str);

  printer_state_msg_.data = state;
  printer_state_pub_.publish(printer_state_msg_);
}

void TaskManager::publishPoseStop()
{
  publishStatus(LOG_LEVEL_T::WARN, "Stopping vehicle.");
  pose_stop_pub_.publish(std_msgs::Empty());
}

void TaskManager::clearPoseTask()
{
  if (pose_task_ptr_.get() != nullptr)
  {
    pose_task_ptr_.release();
    publishStatus(LOG_LEVEL_T::WARN, "Pose task was canceled.");
  }
}

void TaskManager::clearPrinterTask()
{
  if (printer_task_.size() > 0)
  {
    while (!printer_task_.empty())
      printer_task_.pop();
    publishStatus(LOG_LEVEL_T::WARN, "Printer task was canceled.");
  }
}
}