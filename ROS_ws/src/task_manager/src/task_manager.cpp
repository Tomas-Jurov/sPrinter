#include "../include/task_manager.h"

namespace TaskManagerNS
{
  TaskManager::TaskManager(const ros::Publisher& pose_cmd_pub, const ros::Publisher& printer_cmd_pub,
                          const ros::Publisher& printer_state_pub, ros::Publisher& pose_stop_pub,
                          const ros::Publisher& gps_cmd_pub, const ros::Publisher& status_pub)
    : state_(TaskManagerNS::START)
    , watchdog_last_(ros::Time::now())
    , pose_cmd_pub_(pose_cmd_pub)
    , printer_cmd_pub_(printer_cmd_pub)
    , printer_state_pub_(printer_state_pub)
    , pose_stop_pub_(pose_stop_pub)
    , gps_cmd_pub_(gps_cmd_pub)
    , status_pub_(status_pub)
    , pose_task_ptr_(nullptr)
  {
  }

  void TaskManager::update()
  {
    if (ros::Duration(ros::Time::now() - watchdog_last_).toSec() > WATCHDOG_TIMEOUT)
    {
      publishStatus(LOG_LEVEL_T::ERROR, "Task Manager", "Connection lost. Canceling all jobs.");
      ROS_ERROR("Connection lost. Canceling all jobs.");
      safetyStop();
    }
    
    if (state_ == INITIALIZING)
    {
    }
    else if (state_ == ROBOT_READY)
    {
      if (!printer_task_.empty())
      {
        publishStatus(LOG_LEVEL_T::OK, "Task Manager","Printer request state: READY");
        publishPrinterTargetState(1);
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
        publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Printer request state: HOME");
        publishPrinterTargetState(0);
        setState(PRINTER_BUSY);
      }
    }
  }

  void TaskManager::safetyStop()
  {
    pose_task_ptr_.release();
    clearPrinterTask();

    if (state_ != ROBOT_READY)
    {
      if (state_ == ROBOT_MOVING)
      {
        publishPoseStop();
      }
      else if (state_ == PRINTER_BUSY || PRINTER_IDLE)
      {
        publishPrinterTargetState(PrinterState::HOME);
      }
    }
  }

  void TaskManager::heartbeatCallback(const std_msgs::Empty::ConstPtr& msg)
  {
    watchdog_last_ = ros::Time::now();
  }

  void TaskManager::safetyStopCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "SAFETY STOP received! Caneling all jobs.");
    safetyStop();
  }

  bool TaskManager::initializeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if (state_ = START)
    {
      publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Starting a procedure to estimate global orientation.");
      publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "Robot will start to move soon!");
      gps_cmd_pub_.publish(std_msgs::Empty());
      setState(INITIALIZING);
    }
    else
      publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "Robot has been initialized before. Ignoring.");
    
    return true;
  }

  bool TaskManager::poseTaskCallback(sprinter_srvs::SetPose2D::Request& req,
                            sprinter_srvs::SetPose2D::Response& res)
  {
    if (pose_task_ptr_.get() != nullptr)
    {
      pose_task_ptr_ = std::make_unique<geometry_msgs::Pose2D>(req.pose);
      publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Pose task set successfully.");
    }
    else
    {
      publishStatus(LOG_LEVEL_T::OK, "Task Manager", "The previous pose task hasn't been completed yet. Ignoring.");
    }
    
    return true;
  }
  bool TaskManager::printerTaskCallback(sprinter_srvs::SetPointArr::Request& req,
                            sprinter_srvs::SetPointArr::Response& res)
  {
    if (req.points.size() > 0)
    {
      if (printer_task_.empty())
      {
        for (auto &point : req.points)
        {
          printer_task_.emplace(point);
        }
        publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Printer task set successfully.");
      }
      else
      {
        publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "The previous printer task hasn't been completed yet. Ignoring.");
      }
    }
    publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "Got an empty printer task. Ignoring.");
    

    return true;
  }

  void TaskManager::gpsProcessingCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Estimation of global orientation finished.");
    setState(ROBOT_READY);
  }

  void TaskManager::poseControlCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Pose task completed successfully.");
    setState(ROBOT_READY);
  }

  void TaskManager::printerControlCallback(const std_msgs::Int8::ConstPtr& msg)
  {
    if (msg->data == PrinterState::HOME)
    {
      setState(ROBOT_READY);
    }
    else if (msg->data == PrinterState::IDLE)
    {
      if (printer_task_.empty())
      {
        publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Printer task completed successfully.");
        setState(PRINTER_IDLE);
      }
      else
      {
        publishPrinterTargetCmd();
      }
    }
    else if (msg->data == PrinterState::FAILURE)
      publishStatus(LOG_LEVEL_T::ERROR, "Task Manager", 
                    "Printer Control unable to reach target point. Canceling task.");
      clearPrinterTask();
      setState(PRINTER_IDLE);
  }

  void TaskManager::setState(State state)
  {
    state_ = state;
    
    std::string state_str;
    switch (state)
    {
      case START        : state_str = "START"; break;
      case INITIALIZING : state_str = "INITIALIZING"; break;
      case ROBOT_READY  : state_str = "ROBOT READY"; break;
      case ROBOT_MOVING : state_str = "ROBOT MOVING"; break;
      case PRINTER_BUSY : state_str = "PRINTER BUSY"; break;
      case PRINTER_IDLE : state_str = "PRINTER IDLE"; break;
      default: state_str = "UNDEFINED"; break;
    }

    publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Switching to " + state_str + "state.");
  }

  void TaskManager::publishStatus(const int8_t logger_level, const std::string &name,
                                  const std::string& message)
  {
    status_msg_.level = logger_level;
    status_msg_.name = name;
    status_msg_.message = message;
    
    if (!status_msg_.message.empty())
      status_pub_.publish(status_msg_);
  }

  void TaskManager::publishPoseTarget()
  {
    if (pose_task_ptr_.get() != nullptr)
    {
      publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Sending command to Pose Control.");
      publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "Robot will start to move soon!");
      pose_cmd_pub_.publish(*pose_task_ptr_.release());
    }
  }

  void TaskManager::publishPrinterTargetCmd()
  {
    if (!printer_task_.empty())
    {
      publishStatus(LOG_LEVEL_T::OK, "Task Manager", "Sending command to Printer Control.");
      publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "WARNING: Lens will start to move soon!");
      printer_cmd_pub_.publish(printer_task_.front());
      printer_task_.pop();
    }
  }

  void TaskManager::publishPrinterTargetState(const uint8_t state)
  {
    printer_state_msg_.data = state;
    printer_state_pub_.publish(printer_state_msg_);
  }

  void TaskManager::publishPoseStop()
  {
    publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "Stopping vehicle.");
    pose_stop_pub_.publish(std_msgs::Empty());
  }

  void TaskManager::clearPrinterTask()
  {
    while (!printer_task_.empty())
          printer_task_.pop();
    publishStatus(LOG_LEVEL_T::WARN, "Task Manager", "Printer task was canceled.");
  }

}