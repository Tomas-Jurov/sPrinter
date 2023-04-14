#include "../include/task_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_manager");
    ros::NodeHandle nh;

    ros::Publisher vehicle_pub = nh.advertise<geometry_msgs::Pose2D>("target/vehicle/cmd", 1);
    ros::Publisher printer_pub = nh.advertise<geometry_msgs::Point>("target/printer/cmd", 1);
    
    TaskManager task_manager(vehicle_pub, printer_pub);

    ros::Subscriber vehicle_sub = nh.subscribe("target/vehicle/reached", 1, &TaskManager::vehicleFeedback, &task_manager);
    ros::Subscriber printer_sub = nh.subscribe("target/printer/reached", 1, &TaskManager::printerFeedback, &task_manager);

    while (nh.ok())
    {
        ros::spinOnce();
    }
}
