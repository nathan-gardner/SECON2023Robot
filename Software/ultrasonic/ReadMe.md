# ultrasonic

## Software Function

Ultrasonic demo that was created while the team was considering implementing ultrasonic in the project.

## Dependencies

Operating system: 22.04.1-Ubuntu
IDE: VSCode
Extensions: PlatformIO
Required installations: ROS Melodic, rosserial

## How to Install

Step by step instructions for installation can be found below:

[VSCode](https://code.visualstudio.com/Download)

[ROS Melodic Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

[ROSSerial Installation](https://sites.duke.edu/memscapstone/using-rosserial-to-setup-a-ros-node-on-a-teensy/)

## Program Execution

This is a PlatformIO project and code can be uploaded to the Arduino using the upload button (->) on the command bar of at the bottom of the VSCode window. 

![image](https://user-images.githubusercontent.com/30758520/235314336-3c00a6e5-388f-4fdf-834d-0c02e768043e.png)

The following code is executed on the Nvidia Jetson (or comparable computer) to execute this program

```
# start ros master
roscore

# start rosserial
rosrun rosserial_python serial_node.py _port=[PATH_TO_ARDUINO_PORT] _baud=115200
```