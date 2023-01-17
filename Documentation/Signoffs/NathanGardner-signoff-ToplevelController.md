# Top-level Controller Subsystem Signoff

The top-level controller for the robot is the controller that is used as the top level for the ROS network on the robot. All other controllers besides the top level will be Arduino or some other microcontroller, and not have a dedicated full operating system.

## Brief overview of the controller network as a whole

The top level controller will be connected via USB to the two low-level controllers, which will be both Arduino Mega 2560. These two controllers where designed in a previous signoff, one of which is dedicated to locomotion, feeding, fireworks, and consumption. The other low level controller is responsible for sorting and storage of the objects around the arena. 

The first low-level controller will have two peripheral Arduino Nano for flipping the switch for the fireworks and the other for spinning the wheel up to place the food chips in the correct feeding area. 

It will also send the control signals to the motors for the arena locomotion system. It will send PWM signals to the motor drivers to the control its position in the arena. 

The second microcontroller will be used for the sorting and storage subsystem for the pedestals and ducks for the competition. This will include reading a color sensor for which will in turn actuate a solenoid that will push pedestals off the conveyor and will allow the ducks to pass, and read a proximity sensor which will initiate a sequence that will drop off a stack of pedestals, a statue, onto the playing field. 

## Function of the Top-level Controller Subsystem

The top-level controller will serve as the master the the two aforementioned Arduino Mega controllers, and will serve as the hub for the vision subsystem of the robot. The vision subsystem, which will be a network of sensors, will interface with the top-level controller. The vision subsystem is yet to be designed for this project. 

The top-level controller will host the dedicated OS which is required to orchestrate robot operating system (ROS) connections by serving as the master and providing name registration and lookup for the computation graph. The top-level controller will also have a parameter server which is like a large C struct that will be updated by the nodes when values change in the system. 

Devices, namely the two low level controllers, will host nodes that are dedicated processes for sensors or functions of the robot. This is intended to allow for clean separation of code and to allow for scalability of sensor networks and communication for the robot. 

## Constraints

The top-level controller must have two USB type A ports that will be used to connect the the two low-level controllers. THe controller must also communicate with the vision sensor network and retrieve data, process that data, and send commands to actuators around the robot, which will perform actions in the arena.

The controller has a size constraint as does the robot. The top-level controller cannot get too hot and require a large fan to cool, because of the power and space constraints. 

In summary, these constraints are as follows:
- Power
- Size
- GPIO

## Electronic Schematic

## Analysis

## Software Analysis - Processing Capabilities and Possible/Probable Software Analysis

## BOM
