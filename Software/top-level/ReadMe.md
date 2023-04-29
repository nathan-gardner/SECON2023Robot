# top-level

## Software Function

ROS package with holds the script that ran on the Nvidia Jetson to command the robot to follow its path in the arena and actuate motors.

## Dependencies

Operating system: 22.04.1-Ubuntu
Required installations: ROS Melodic, rosserial

## How to Install

[ROS Melodic Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

[ROSSerial Installation](https://sites.duke.edu/memscapstone/using-rosserial-to-setup-a-ros-node-on-a-teensy/)

## Program Execution

The following code is executed on the Nvidia Jetson (or comparable computer) to execute this program

```
# start ros master
roscore

# start rosserial
rosrun rosserial_python serial_node.py _port=[PATH_TO_ARDUINO_PORT] _baud=115200

# to start main top_level.py execution, navigate to top_level.py and run with python command
cd [PATH_TO_REPO]/Software/top-level/top-level-package/scripts
python top_level.py
```