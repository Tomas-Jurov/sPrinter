# sPrinter ROS workspace

### Packages list
* [task_manager](./src/task_manager/README.md)
* [pose_control](./src/pose_control/README.md)
* [printer_control](./src/printer_control/README.md)
* [state_estimation](./src/state_estimation/README.md)
* [ros_bridge](./src/ros_bridge/README.md)
* [gps](./src/gps/README.md)

### Structure overview
```
sPrinter/ROS_ws
|_src
  |_task_manager
  | |_include
  | | |_task_manager.h          -- headder file
  | |_src
  | | |_task_manager_node.cpp   -- node interface (node handle, topic publications, subscriptions)
  | | |_task_manager.cpp        -- main implementation
  | |_README.md                 -- describe the package here
```

## Build
* build one package
    ```
    $ catkin build task_manager
    ```

* build the whole workspace
    ```
    $ catkin build
    ```

* Note 1: catkin build requires python catkin tools installed (`sudo apt install python3-catkin-tools`)  
* Note 2: before building the whole workspace, follow [Install dependencies for GPS](./src/gps/README.md) steps or run
    ```
    $ catkin config --skiplist gps
    ``` 

## Launch
* Run single node (example)
    ```
    $ . ROS_ws/devel/setup.bash
    $ rosrun task_manager task_manager
    ```
* Run multiple nodes  
    See [sprinter/README.md](./src/sprinter/README.md)