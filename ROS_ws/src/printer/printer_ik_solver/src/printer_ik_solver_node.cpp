#include "../include/printer_ik_solver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "printer_ik_solver");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("sprinter_status", 50);
  ros::ServiceClient validity_client = nh.serviceClient<moveit_msgs::GetStateValidity>("check_state_validity");
  PrinterIKSolver printer_ik_solver(status_pub, validity_client);

  ros::ServiceServer get_ik_service =
      nh.advertiseService("ik/get_solution", &PrinterIKSolver::calculateIkService, &printer_ik_solver);

  ros::Rate loop_rate(40);
  while (ros::ok())
  {
    loop_rate.sleep();
  }
}
