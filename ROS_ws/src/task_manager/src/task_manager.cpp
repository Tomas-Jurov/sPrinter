#include "../include/task_manager.h"

void TaskManager::Do()
{
    geometry_msgs::Pose2D vehicle_target;
    geometry_msgs::Point printer_target;

    /* ... */

    vehicle_target_pub_.publish(vehicle_target);
    printer_target_pub_.publish(printer_target);
}

void TaskManager::vehicleFeedback(const std_msgs::Empty &msg)
{

}

void TaskManager::printerFeedback(const std_msgs::Empty &msg)
{

}
