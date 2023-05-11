# State Estimation package

## Description
Publishes TF messages according to feedback from motors and sensoric data.

---

## Testing odom functionality
To test the state estimation node functionality you can publish twist message on rostopic. \
In `RViz` set `Fixed  Frame` to `odom`.

Example message to publish:
```
rostopic pub wheels/twist geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

---

### TF tree
<p align="center">
    <img src="../../../doc/sPrinter_design_diagram-TF_tree.svg">
</p>

## Setup


## Usage
