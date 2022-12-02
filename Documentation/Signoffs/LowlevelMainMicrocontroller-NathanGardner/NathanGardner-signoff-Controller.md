# Low-level Controller Subsystem Signoff

## Function of the Subsystem

The specific function of the microcontroller network described in this sign off is operating as the main level hardware access controller. This means that this device is responsible controlling the logic to drive subsystem motors across the subsystem. The main top-level microcontroller, which will be designed later and is not this subsystem, is what will be responsible for vision, robot path plan storage and logic to follow path, and most importantly be the ROS controlling device. 

The robot will be controlled via a microcontroller (MCU). The main constraints on the microcontroller subsystem are general purpose input/output (GPIO) constraints as well as computational constraints for location, vision communication, and object manipulation logic constraints. The microcontroller will generate Pulse-Width Modulation (PWM), digital, and interrupt signals to the various motor drivers and systems contained within the locomotion subsystem. 

The microcontroller subsystem is essentially the brains of the robot, and consideration is taken below for how fast the robot needs to "think" or retrieve, process, and store data. In most cases in this implementation, this will be designed to mitigate issues with the real-time requirements of the robot. Issues could arise from not being able to gather and process information quickly enough from GPIO that is driven by a peripheral sensor. This subsystem is dependant on the sensors chosen for the design, but worst case constraints will be assumed in many cases (more GPIO than required, higher processing speed then required, etc.) so that the requirements are met in the final design. 

## Constraints

The controller subsystem has constraints that it must abide by to be successful. The first and foremost is GPIO limitations, we will need enough GPIO to drive all the motors and read sensors and to interface with any other microcontrollers in the design. Power consumption will also be a constraint because the power system of the robot cannot be unnecessarily stressed in terms of power drawn from the controller. 

The direct interfaces between the controller and other subsystems are as follows:

1. Serial USB communication to the vision microcontroller. This will be a general USB port used to communicate objects which describe environment.
2. An input from the power system which triggers the microcontroller to start initializing code. This will be a digital input pin with hardware interrupt capabilities. 
3. Serial USB communication as an input to the object storage subsystem. This will be a output in terms of the microcontroller. 
4. PWM and digital logic high/low communication as an input to the object consumption subsystem. This will be a digital output pin in terms of the microcontroller.
5. The power input to the microcontroller, which will come from the power regulator in the power subsystem.  

Standard: IEEE 1118.1-1990 describes standards related to interdevice/intrabuilding as well as interconnection of microcontrollers. This standard will be reference and adhered to in terms of system controller interconnect design, which is part of the controller subsystem. 

Conceptual Design Document: [here](https://github.com/nathan-gardner/CapstoneRepo/blob/main/Reports/Team2_ConceptualDesignandPlanningFinal.pdf)

Number of motors that need to be driven and sensors that need to be read by the main controller and what purpose they serve are listed below

### Two Low-level Mega Controllers (Locomotion, Fireworks, Feeding, Consumption and Sorting, Storage)

A design with two Arduino Mega 2560 Rev3 microcontrollers were selected in order to create a modular design and to all for parallelization of the design amongst the team members. This design also allows up to come in well above the general IO requirement for each controller, which is covered in great detail below. 

Feeding subsystem - one motor that needs to be driven by the main microcontroller

| Feeding                     |           |       |             |   |
|-----------------------------|-----------|-------|-------------|---|
|                             | Pin Type  | Count |             |   |
| Activation for Arduino Nano | Interrupt | 1     |             |   |
|                             |           |       | Interrupt:  | 1 |
|                             |           |       | Total Pins: | 1 |

_Note: This Arduino Nano will need to be designed as part of the microcontroller network but will not be designed until the Feeding Subsystem is designed and will reflect the specific requirements of the Feeding Subsystem. It will be part of the Feeding signoff but will be part of the low-level controller network. This subsystem is standalone and a lower priority portion of the design. Right now all that is known is that the main Mega controller will generate a hardware interrupt which will invoke an interrupt service routine on the Nano._

_Interfaces needed:_ 
1. Motor for the rack and pinion subsystem

Fireworks subsystem - zero or one motor to flip the switch for the fireworks

| Fireworks                   |           |       |             |   |
|-----------------------------|-----------|-------|-------------|---|
|                             | Pin Type  | Count |             |   |
| Activation for Arduino Nano | Interrupt | 1     |             |   |
|                             |           |       | Interrupt:  | 1 |
|                             |           |       | Total Pins: | 1 |

_Note: This Arduino Nano will need to be designed as part of the microcontroller network but will not be designed until the Fireworks Subsystem is designed and will reflect the specific requirements of the Fireworks Subsystem. It will be part of the Fireworks signoff but will be part of the low-level controller network. This subsystem is standalone and a lower priority portion of the design. Right now all that is known is that the main Mega controller will generate a hardware interrupt which will invoke an interrupt service routine on the Nano._

_Interfaces needed:_ 
1. Servo control signal for flipping a switch if needed, will design to that need 

Locomotion subsystem - This subsystem will require four motors driven independently. We will need to use both encoder output A and B on all four motors to show direction of rotation. The table of the motor inputs that need to be driven by the motor are below.

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

Consumption Subsystem - Subsystem which will consume objects around the arena. Will be required to drive the motor driver for the object consumption subsystem. 

| Consumption  |          |       |             |   |
| ------------ | -------- | ----- | ----------- | - |
| Motor Driver | Pin Type | Count |             |   |
|              | Digital  | 2     |             |   |
|              | PWM      | 1     |             |   |
| Motor Driver |          |       |             |   |
|              | Digital  | 2     |             |   |
|              | PWM      | 1     |             |   |
|              |          |       | Digital:    | 4 |
|              |          |       | PWM:        | 2 |
|              |          |       | Total Pins: | 5 |

_Interfaces needed:_ 
1. PWM to control the speed of the motor that drives the object consumption system
2. Two digital pins to Forward and Backward Directions
3. Two digital pins to read the encoder outputs to get accurate measurements for speed and direction

Vision Subsystem - Requires communication to exchange sensor data structures through serial USB communication. The vision subsystem is designed to abstract away the complication of the sensor data and path planning and communicate with the main controller subsystem lower level controllers, designed in this signoff, which will execute commands to drive motors and drive actuators. 

The vision system will house all of the sensor for data acquisition for the robot and will communicate with the top-level main microcontroller. The top-level main microcontroller will communicate with the low-level main microcontroller via serial USB. The top-level microcontroller will have a dedicated operating system (likely some Linux distribution) and will act as the master where the two low-level microcontrollers, designed in this sign off, will act as the slaves. The USB will allow master and slave to communicate with each other serially with universal asynchronous transmitter receiver (UART). 

_Total pins needed:_
| Locomotion, Fireworks, and Feeding, Consumption Controller |    |
|------------------------------------------------------------|----|
| Power:                                                     | 2  |
| Digital:                                                   | 12 |
| PWM:                                                       | 7  |
| Interrupt:                                                 | 2  |
| Total:                                                     | 17 |

### Object Sorting and Storage Controller

Object Sorting - belt motor and a motor or pressurized air which will poke the pedestals off of the belt and into a separate area.

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
| Negative           | Power     | 1     |             |    |
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
| Communication    | Micro-USB | 0 |             |    |
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

The Arduino Mega 2560 Rev3 was analyzed for selection as the main controller and the object sorting and storage controller. The board has 54 digital pins, 15 of which can be used as PWM outputs, 16 analog inputs, 4 UARTS, and a USB connection.

The Arduino Mega can integrate ROS, and ROS will definitely be used in this project. ROS libraries will be used for hardware abstraction, low-level device control, and package management.

The Arduino Mega was selected for these two microcontrollers because it has a comfortable cushion for the I/O constraint, and allows us to separate and design the controller system modularly. 

Two microcontrollers are being used so that the design is modular and can be split up in a way that makes sense. The controllers are mostly driving motor controllers with PWM and digital outputs, and those actions are not computationally expensive.

The object sorting has a color sensor which will send data at $400\ \frac{kbits}{sec}$, and the Arduino Mega, and the clock of the microcontroller is 16 MHz and with the pre-scale set to 128 by default, this means that the digital pins can be sampled at $150\ \frac{k-bits}{sec}$. The the 128 pre-scale can be changed, so this can be updated to our requirements. Analysis is shown below.

The object storage subsystem has proximity sensors which outputs at 145 Hz, which will be sufficient for our use case of only needing to know when the silos are full. This is not something that needs to be sampled at an extremely high speed. Analysis is shown below. 

## Electronic Schematic (this is out of date because of fireworks and feeding pin changes)

The electronic schematic for the controller subsystem is attached below. It includes the main microcontroller and the interface to each of the subsystems which they require to drive actuators. 

![image](https://user-images.githubusercontent.com/30758520/203182166-b2f8a488-171c-4eee-b01a-84e9ea9c25ab.png)

Electronic Schematic Files: [here](https://github.com/nathan-gardner/CapstoneRepo/tree/NathanGardner-signoff-Controller/Documentation/Electrical/Schematics/Sources)

## Analysis

The analysis below is used to show why the Arduino Mega microcontroller is going to be be used for the main controller subsystem.

Tables are shown in the constraints section and contain pin count analysis. The pins are one of the major constraints on the design for this subsystem. 

### Color Sensor in Object Sorting

The belt will be moving at $2 \frac{inches}{sec}$

The calculations for the samples per unit distance are below:

$v_{belt} = 2\ \frac{inches}{sec}$

$f_{color\ sensor} = 400\ \frac{ksamples}{sec}$

$f_{arduino} = 150\ \frac{ksamples}{sec}$

$f_{effective} = 75\ \frac{ksamples}{sec}$

$\frac{samples}{inch} = \frac{1\ sec}{2\ inches} \ast \frac{75\ ksamples}{1\ sec} = 37.5\ \frac{ksamples}{sec}$

Calculations for the frequency of objects recieved:

$f_{duck} = \frac{80\ sec}{10\ ducks} = 8 \frac{sec}{duck}$

$f_{pedestal} = \frac{80\ sec}{7\ pedestals} = 11.4 \frac{sec}{pedestals}$

$f_{object} = \frac{80\ sec}{17\ objects} = 4.7 \frac{sec}{object}$

$f_{objects\ received\ on\ belt} = \frac{17\ objects}{80\ sec} =  0.2125\ Hz$

This means that the sample rate of $37.5\ \frac{ksamples}{sec}$ is more than enough for sampling the the color of the objects coming in on the belt. The robot will actually be able to sample a pedestal many times, the calculations for that are below:

$\frac{samples}{L_{pedestal}} = 75\ \frac{ksamples}{sec} \ast \frac{1 sec}{2 inches} \ast \frac{2 inches}{1\ L_{pedestal}}$

Nyquist rate for the color sensor sampling:

$f_{color\ sample} = 2 \ast 2\ Hz = 4\ Hz$

### Proximity Sensor in Object Storage

The silo will need to sampled twice a second, so the actual sampling will be happening at 4 Hz, by Nyquists Theorem.

$f_{proximity\ sample} = 2 \ast 2\ Hz = 4\ Hz$ by Nyquist Theorem.

## Software Analysis - Processing Capabilities and Possible/Probable Software Analysis

Arduino public C++ libraries will be used for these microcontrollers. We will use the built in Arduino PWM functionality in this library to interface with the motors (PWM) and read and write digital pins on the Arduino Mega. Part of the reason why Arduino was chosen as the low level microcontroller is the amount of public libraries available for the devices. Public libraries that will very likely be used in this controller implementation are below.

The **[Core Library Used for Arduino Mega 2560 Rev3](https://github.com/arduino/ArduinoCore-avr)** will be necessary for this project, as it is the functions provided by Arduino and in the IDE and is what most peripheral libraries have dependencies on. 

***Some* Specific Software Implementations:**

[Servo](https://github.com/arduino-libraries/Servo/tree/master/src/avr) will be used to drive servo motors and is a public library provided by Arduino. 

[Digital Read](https://github.com/arduino/ArduinoCore-avr/blob/42fa4a1ea1b1b11d1cc0a60298e529d37f9d14bd/cores/arduino/wiring_digital.c#L165) will read pins and determine if they are logic high or low. 

[Digital Write](https://github.com/arduino/ArduinoCore-avr/blob/42fa4a1ea1b1b11d1cc0a60298e529d37f9d14bd/cores/arduino/wiring_digital.c#L138) will set output pins as logic high or low.

[Analog Read](https://github.com/arduino/ArduinoCore-avr/blob/42fa4a1ea1b1b11d1cc0a60298e529d37f9d14bd/cores/arduino/wiring_analog.c#L38) will be used to read PWM coming from the encoder, which will return the speed to the main microcontroller. 

[Analog Write](https://github.com/arduino/ArduinoCore-avr/blob/42fa4a1ea1b1b11d1cc0a60298e529d37f9d14bd/cores/arduino/wiring_analog.c#L96) will be used to write analog values and to implement PWM functionality to drive DC motors in various subsystems.

[PWM](https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_analog.c) will be implemented using the analog read and write functions in the public Arduino library.

The Arduino Mega 2560 Rev3's clock runs are 16 MHz using the ATmega2560 microcontroller onboard. The board has 256 KB of flash memory, 8 KB of SRAM, and 4 KB of EEPROM memory. Using the design outlined, be are using a small percentage of the GPIO on the Arduino Mega, because of the modular design. The source code for each project will be compiled so it will be lightweight on flash memory for the Arduino Mega 2560 Rev3. The modular design allows the team to be comfortable that we will not reach computational limitations due to to many peripheral devices connected to a single device. 

The Arduino will not run a dedicated OS, but instead has a basic bootloader that performs hardware and software initialization and then jump to the main method in the code. PlatformIO will likely be used as the IDE for this project because of its extra functionality in importing dependencies and integration into VScode as an extension. PlatformIO has 650 boards available including the Arduino Mega 2560 Rev3 so it has been chosen for is versatility. 

## BOM

| Name of Item               | Description                                          | Used in which subsystem(s) | Part Number           | Manufacturer     | Quantity | Price      | Total  |
|----------------------------|------------------------------------------------------|----------------------------|-----------------------|------------------|----------|------------|--------|
| Arduino Mega 2560 Rev3     | Microcontrollers selected for controller subsystem   | Controller                 | A000067/7630049200067 | Arduino          | 2        | 48.4       | 96.8   |
| Jumper Wires ELEGOO 120pcs | Jumper wires to connect board to sensors and motors | Controller                 | B01EV70C78            | ELEGOO           | 1        | 9.99       | 9.99   |
| Total                      |                                                      |                            |                       | Total Components | 3        | Total Cost | 106.79 |
