# Task Manager package

## Description
The `task_manager` node processes input commands from the `sprinter_control` node into commands for the `pose_control`, `printer_control` and `gps_processing` nodes, securing correct operation of the robot.

### States
 * **START**
    - turned on
    - onboard and remote ROS nodes are launched
    - odometry is zero
 * **INITIALIZING**
    - performing procedure to acquire initial orientation in the `world` frame
 * **ROBOT READY**
    - robot standstill
    - `printer_control` state is "home"
    - ready to receive pose or printer commands
 * **ROBOT MOVING**
    - `pose_control` node is moving the robot to the target point
 * **PRINTER IDLE**
    - `printer_control` state is "ready"
 * **PRINTER BUSY**
    - `printer_control` node is moving from/to "home" position or performing the printing procedure

<p align="center">
    <img src="../../../doc/sPrinter_design_diagram-Task_manager_states.svg" width="400">
</p>
<p align="center">
    Task manager states flowchart
</p>

<p align="center">
    <img src="../../../doc/sPrinter_design_diagram-Task_manager_program.svg">
</p>
<p align="center">
    Task manager program flowchart
</p>

## Setup


## Usage
