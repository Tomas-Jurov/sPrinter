#include "../include/printer_ik_solver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "printer_ik_solver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
//    ros::waitForShutdown();



    ros::Publisher status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("sprinter_status", 50);
    PrinterIKSolver printer_ik_solver(status_pub);

    ros::ServiceServer get_ik_service =
            nh.advertiseService("ik/get_solution", &PrinterIKSolver::calculateIkService, &printer_ik_solver) ;



    while (ros::ok())
    {
    }
}
