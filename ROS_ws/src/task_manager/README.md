# Task Manager package

## Description
The `task_manager` node processes input commands from the `sprinter_control` node into commands for the `pose_control`, `printer_control` and `gps_processing` nodes, securing correct operation of the robot.

### States
   * **START**
      - onboard and remote ROS nodes are launched
      - odometry is zero
   * **INITIALIZING**
      - performing procedure to acquire initial orientation in the `world` frame
   * **ROBOT READY**
      - robot standstill
      - lens folded
      - ready to receive pose or printer commands
   * **ROBOT MOVING**
      - `pose_control` node is moving the robot to the target point
   * **PRINTER IDLE**
      - `printer_control` ready to receive commands
      - lens unfolded
   * **PRINTER BUSY**
      - `printer_control` node is moving from/to "home" position or performing the printing procedure
   * **STOPPING**
      - reaching the safe state (ROBOT READY)
   

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

## Usage
To work, it needs a connetion with running `sprinter_control` node. Otherwise, after timeout, the watchdog functionality will keep the robot in the safe state.
`WATCHDOG_TIMEOUT` is set in the `task_manager.h` file, `HEARTBEAT_INTERVAL` is set in the `sprinter_control.h` file.

   * **initialize** (start the GPS measuriong procedure)
```bash
   $ rosservice call /task_manager/initialize '{}'
```
   * **set a pose task**
```bash
   $ rosservice call /task_manager/set_task/pose "{pose: {x: 1.0, y: 2.0, theta: 3.0}}"
```
   * **set a printer task**
```bash
   $ rosservice call /task_manager/set_task/printer '{points:[{x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}]}'
```
