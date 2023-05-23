#include "../include/printer_ik_solver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "printer_ik_solver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
//    ros::waitForShutdown();

    PrinterIKSolver printer_ik_solver; //err

    ros::ServiceServer get_ik_service =
            nh.advertiseService("ik/get_solution", &PrinterIKSolver::calculateIkService, &printer_ik_solver) ;



    while (ros::ok())
    {
    }
}
