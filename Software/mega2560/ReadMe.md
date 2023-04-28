# mega2560/mega1

## Software Function

PlatformIO Project that ran on the Arduino Mega during the competition. Mega functioned as the main motor controller and hardware interface device for this project.

## Dependencies

Operating system: 22.04.1-Ubuntu
IDE: VSCode
Extensions: PlatformIO
Required installations: ROS Melodic

## How to Install

Tutorials and links for installation can be found below:

[VSCode](https://code.visualstudio.com/Download)

[ROS Melodic Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

[ROSSerial Installation](https://sites.duke.edu/memscapstone/using-rosserial-to-setup-a-ros-node-on-a-teensy/)

## Program Execution

This is a PlatformIO project and code can be uploaded to the Arduino using the upload button (->) on the command bar of at the bottom of the VSCode window. 

![image](https://user-images.githubusercontent.com/30758520/235266512-2e0a4d6e-eff6-4ddb-b949-f5bebe7ae6d1.png)

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


