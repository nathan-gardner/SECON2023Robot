# Test_ROSSerial

## Software Function

Initial PlatformIO project created in testing. This was used to setup ROSSerial and to confirm that we would be able to use ROSSerial to pass messages between the Nvidia Jetson and the Arduino Mega as we had planned in design.

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

![image](https://user-images.githubusercontent.com/30758520/235264506-e5d24a43-ba47-4403-815a-91b4373e0341.png)

The following code is executed on the Nvidia Jetson (or comparable computer) to execute this program

```
# start ros master
roscore

# start rosserial
rosrun rosserial_python serial_node.py _port=[PATH_TO_ARDUINO_PORT] _baud=115200
```



