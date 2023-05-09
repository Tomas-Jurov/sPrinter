# sPrinter Control package

## Description
User control interface for sPrinter ROS implementation.

## Setup


## Usage
* Source the environment
    ```
    $ . ROS_ws/devel/setup.bash
    ```
* Launch mid-level control (on the onboard RPI device)
    ```
    $ roslaunch sprinter_control mid_level.launch
    ```
* Launch high-level control (on remote PC)
    ```
    $ roslaunch sprinter_control high_level.launch
    ```
