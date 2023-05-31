# Printer Control package

## Description
This node computes Inverse Kinematics for robot sPrinter, especially for lens with joints belonging to `lens_group`. 
The node serves as rosservice server for `ik/get_solution`, where the request is desired pose of end-effector for printing in
`base_link` frame and the response are joint states for given pose calculated via MoveIt!.

## Setup
Pre requisite:
```bash
sudo apt install ros-noetic-moveit
```

## Usage
To launch the printer control node, type
```bash
roslaunch printer_ik_solver printer_ik_solver.launch
```
