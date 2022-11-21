# Controller Subsystem Signoff

## Function of the Subsystem

The robot will be controlled via a microcontroller (MCU). The main constraints on the microcontroller subsystem are general purpose input/output (GPIO) constraints as well as computational constraints for location, vision communication, and object manipulation logic constraints. 

The microcontroller subsystem is essentially the brains of the robot, and consideration is taken below for how fast the robot needs to "think" or process retrieve, process, and store data. In most cases in this implementation, this will be designed to mitigate issues with the real-time requirements of the robot. Issues could arise from not being able to gather and process information quickly enough from GPIO that is driven by a peripheral sensor. This subsystem is dependant on the sensors chosen for the design, but worst case constraints will be assumed in many cases (more GPIO than required, higher processing speed then required, etc.) so that the requirements are met in the final design. 

## Constraints

The controller subsystem has constraints that it must abide by the be successful. The first and foremost is GPIO limitations, we will need enough GPIO to drive all the motors and to interface with any other microcontrollers in the design. Power consumption will also be a constraint because the power system of the robot cannot be unnecessarily stressed in terms of power drawn from the controller. 

The direct interfaces between the controller and other subsystems are as follows:

1. Serial communication input/output to the vision microcontroller. This will be a general digital I/O pin used to communcate objects which describe enviroment.
2. An input from the power system which triggers the microcontroller to start initializing code. This will be a digital input pin with hardware interrupt capabilities. 
3. Serial communication as an input to the object storage subsystem. This will be a digital output pin in terms of the microcontroller. 
4. Serial communication as an input to the object consumption subsystem. This will be a digital output pin in terms of the microcontroller.
5. The power input to the microcontroller, which will come from the power regulator in the power subsystem.  

Standard: IEEE 1118.1-1990 describes standards related to interdevicehtrabuilding as well as interconnection of microcontrollers. This stardard will be reference and adhered to in terms of system controller interconnect design, which is part of the controller subsystem. 

Conceptual Design Document: [here](https://github.com/nathan-gardner/CapstoneRepo/blob/main/Reports/Team2_ConceptualDesignandPlanningFinal.pdf)

Number of motors that need to be driven by the main controller and what purpose those motor serve are listed below

### Main Controller

Feeding subsystem - one motor that needs to be driven by the main microcontroller

| Feeding            |          |       |             |   |
|--------------------|----------|-------|-------------|---|
| Motor Driver       | Pin Type | Count |             |   |
| Speed              | PWM      | 1     |             |   |
| Forward Direction  | Digital  | 1     |             |   |
| Backward Direction | Digital  | 1     |             |   |
|                    |          |       | Digital:    | 2 |
|                    |          |       | PWM:        | 1 |
|                    |          |       | Total Pins: | 3 |

_Interfaces needed:_ 
1. Motor for the rack and pinion subsystem

Fireworks subsystem - zero or one motor to flip the switch for the fireworks

Still undesigned, in the worst case this will need s control signal for a servo

| Fireworks |          |       |             |   |
|-----------|----------|-------|-------------|---|
| Servo     | Pin Type | Count |             |   |
| Power     | Power    | 1     |             |   |
| Ground    | Power    | 1     |             |   |
| Control   | PWM      | 1     |             |   |
|           |          |       | Power:      | 2 |
|           |          |       | PWM:        | 1 |
|           |          |       | Total Pins: | 3 |

_Interfaces needed:_ 
1. Servo control signal for flipping a switch if needed, will design to that need 

Locomotion subsystem - This subsystem will require four motors driven independantly. We will need to use both encoder output A and B on all four motors to show direction of rotation. The table of the motor inputs that need to be driven by the motor are below:

| Locomotion         |          |       |             |    |
|--------------------|----------|-------|-------------|----|
| Motor Driver       | Pin Type | Count |             |    |
| Speed              | PWM      | 4     |             |    |
| Forward Direction  | Digital  | 4     |             |    |
| Backward Direction | Digital  | 4     |             |    |
|                    |          |       | PWM:        | 4  |
|                    |          |       | Digital:    | 8  |
|                    |          |       | Total Pins: | 12 |

This means that four PWM pins will be needed and eight digital pins will be needed for the locomotion subsystem. 

_Interfaces needed:_ 
1. Control signal for four wheels, one PWM signal for speed, one for forward and one for backward direction, or twelve pins in total

Vision Subsystem - Requires communication to exchange sensor data structures through serial USB communcation. The vision subsystem is designed to abstract away the complication of the sensor data and path planning and communcate with the main controller subsystem lower level controllers, which will execute commands to drive motors and drive actuators. 

The vision subsystem will not have any direct pin requirements, but it will use USB communcation to exchange data between the main microcontroller and the vision microcontroller. 

_Total pins needed:_
| Locomotion, Fireworks, and Feeding Controller |    |
|-----------------------------------------------|----|
| Power:                                        | 2  |
| Digital:                                      | 10 |
| PWM:                                          | 6  |
| Total:                                        | 18 |

### Object Sorting and Storage Controller

Object Sorting - belt motor and a motor or pressurized air which will poke the pedestals off of the belt and into a seperate area.

| Object Sorting     |           |       |             |    |
|--------------------|-----------|-------|-------------|----|
| Motor Driver       | Pin Type  | Count |             |    |
| Speed              | PWM       | 1     |             |    |
| Forward Direction  | Digital   | 1     |             |    |
| Backward Direction | Digital   | 1     |             |    |
| Color Sensor       |           |       |             |    |
| Ground             | Power     | 1     |             |    |
| Interrupt          | Interrupt | 1     |             |    |
| SCL                | Digital   | 1     |             |    |
| SDA                | Digital   | 1     |             |    |
| VDD                | Power     | 1     |             |    |
| Vibration Motor    |           |       |             |    |
| Positive           | Power     | 1     |             |    |
| Negitive           | Power     | 1     |             |    |
| Activation         | Digital   | 1     |             |    |
| Linear Actuator    |           |       |             |    |
| Retract            | Digital   | 1     |             |    |
| Extend             | Digital   | 1     |             |    |
|                    |           |       | Interrupt:  | 1  |
|                    |           |       | PWM:        | 1  |
|                    |           |       | Power:      | 4  |
|                    |           |       | Digital:    | 7  |
|                    |           |       | Total Pins: | 13 |

_Interfaces needed:_ 
1. Motor to drive conveyor 
2. Color sensor
3. Vibration motor
4. Linear actuator

Object Storage - 

The pins for the object sorting subsystem are listed below in tabular form. The first motor driver is for the rack and pinion system, and two proximity sensors will be in the silos and will determine when they are full. Finally, the last motor driver inputs will be for the mechanism to open the silos when a statue needs to be dropped off. 

| Object Storage   |           |   |             |    |
|------------------|-----------|---|-------------|----|
| Servo Controller |           |   |             |    |
| Communcation     | Micro-USB | 0 |             |    |
| Ground           | Power     | 1 |             |    |
| Vin              | Power     | 1 |             |    |
| Rx               | Digital   | 1 |             |    |
| Tx               | Digital   | 1 |             |    |
| Reset            | Digital   | 1 |             |    |
| Proximity Sensor |           |   |             |    |
| Vin              | Power     | 2 |             |    |
| Ground           | Power     | 2 |             |    |
| Out              | Digital   | 2 |             |    |
|                  |           |   | Power:      | 6  |
|                  |           |   | Digital:    | 5  |
|                  |           |   | Total Pins: | 11 |

_Interfaces needed:_ 
1. Motor for rack and pinion
2. two proximity sensors
3. some motor for opening the silo

_Total pins needed:_
| Object Storage and Sorting Controller |    |
|---------------------------------------|----|
| Power:                                | 10 |
| Digital:                              | 12 |
| PWM:                                  | 1  |
| Interrupt:                            | 1  |
| Total:                                | 24 |

## Electronic Schematic 

The electronic schematic for the controller subsystem is attached below. It includes the main microcontroller and the interface to each of the subsystems which they require to drive actuators. 



## Analysis

The analysis below is used to show that the Arduino Mega microcontroller is going to be be used for the main controller subsystem.

Tables are shown in the constraints section and contain pin count analysis. 

## BOM
