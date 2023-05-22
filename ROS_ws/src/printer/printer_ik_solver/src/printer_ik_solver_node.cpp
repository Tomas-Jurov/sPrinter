//
// Created by jakub on 22.05.23.
//

#include "../include/printer_ik_solver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "printer_ik_solver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PrinterIKSolver printer_ik_solver(); //err

    while (ros::ok())
    {

    }
}
