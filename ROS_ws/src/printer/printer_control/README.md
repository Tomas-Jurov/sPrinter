# Printer Control package

## Description

### General
Controls movement of lens_group's joints. 
Group **lens_group** includes linear actuator, both stepper motors and both servos to position the lens.

The general user-friendly notification about printer processes are send on topic `sprinter_status` ("Printer Moving to HOME" and so on).

The whole control system has its internal states: HOME, IDLE, FAILURE, BUSY, INIT, PRINTING, IDLE2 or BUSY_TO_IDLE, 
but only BUSY, IDLE and FAILURE is published to Master node - TASK MANAGER. 

In overall, `Task Manager node controls` the Printer Control node. Task manager can send only two states 
on topic `target/printer/state` which will be processed by Printer Control node. `HOME` serves also as error command, 
where the lens must be folded (to home position). Therefore, the HOME state is processed in anytime. 

The second state `IDLE` is processed only when printer is in state HOME.
If printer is in state IDLE, the Task Manager can send printing point on topic `target/printer/cmd`. 
If so, service to Printer IK Solver node is called and Printer Control awaits response. Success means moving 
to state PRINTING to joint state positions obtained from service call. If failure and IK solver couldn't find IK solution,
the FAILURE state is set to printer and Task manager should handle it.

If IK solver returns joint state positions for printing, PRINTING is started. The duration is preset and user is informed about remaining time.

If printing commands are not sent, printer will move its servo1 (on axis x) away from the sun in order to not print anything.
This state is marked as IDLE2 internally, but when reached position, IDLE state is sent on topic `printer_control/state`.

### Control
If set target position for joint states, several update-functions are called.
The order of updating actuators depends on the type of movement. We do not want to move servos if steppers are in home position etc.
The blocker condition is set in update functions in order to not move the actuators until prerequisites for movement are met.

Here is the list of control functions with blocker sent to servos that steppers have to be on its target positions.
- stepper2Update();
- stepper1Update();
- linActuatorUpdate();
- servo2Update(steppersOnPos());
- servo1Update(steppersOnPos());

As shown, we use the all potential of the robot by moving several actuator simultaneously if possible.

The servos are controlled on absolute position, so only desired angle is set.
Steppers work in open-loop control system with no feedback, hence the relative target (the distance to translate the lens) is sent.
Linear actuator does not provide any feedback also and since we didn't know the actual position of the linear actuator, 
we decided to implement IMU from which we compute pitch angle of the robot. Moreover, there is only speed control for the linear actuator,
so we do not set the relative or absolute joint distance to be taken, but the absolute speed is sent. 
This causes the need of the control loop (in contrast to other actuator where just target was only once sent).

## Setup
There are no special requirements for this package.

## Usage
To launch the printer control node, type 
```bash
roslaunch printer_control printer_control.launch
```

We also recommend to use rqt to publish messages on topic if the user wants to test the logic behind Printer Control node. 

For further testing, use also joint_states_gui for fake joint states feedback and display the overal movement in RViz.
