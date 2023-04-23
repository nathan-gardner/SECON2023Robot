# Software

Project source code is stored here. Below is an explanation of each file and how it functioned in the project.

Further documentation on each project can be found in comments/doxygen in the source code. 

## **Files**
- [demo](/Software/demo/) - Initial PlatformIO project created in testing. This was used to setup ROSSerial and to confirm that we would be able to use ROSSerial to pass messages between the Nvidia Jetson and the Arduino Mega as we had planned in design.
- [matlab-sim](/Software/matlab-sim/) - File for MATLAB simulations used during signoffs
- [mega2560](/Software/mega2560/) - PlatformIO Project that ran on the Arduino Mega during the competition. Mega functioned as the main motor controller and hardware interface device for this project.
- [top-level](/Software/top-level/) - ROS package with holds the script that ran on the Nvidia Jetson to command the robot to follow its path in the arena and actuate motors.
- [ultrasonic](/Software/ultrasonic/) - Ultrasonic demo that was created while the team was considering implementing ultrasonic in the project.