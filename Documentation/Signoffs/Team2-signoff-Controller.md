# Controller Subsystem Signoff

## Function of the Subsystem

The robot will be controlled via a microcontroller (MCU). The main constraints on the microcontroller subsystem are general purpose input/output (GPIO) constraints as well as computational constraints for location, vision communication, and object manipulation logic constraints. 

The microcontroller subsystem is essentially the brains of the robot, and consideration is taken below for how fast the robot needs to "think" or process retrieve, process, and store data. In most cases in this implementation, this will be designed to mitigate issues with the real-time requirements of the robot. Issues could arise from not being able to gather and process information quickly enough from GPIO that is driven by a peripheral sensor. This subsystem is dependant on the sensors chosen for the design, but worst case constraints will be assumed in many cases (more GPIO than required, higher processing speed then required, etc.) so that the requirements are met in the final design. 

## Constraints

The controller subsystem has constraints that it must abide by the be successful. The first and foremost is GPIO limitations, we will need enough GPIO to drive all the motors and to interface with any other microcontrollers in the design. Power consumption will also be a constraint because the power system of the robot cannot be unnecessarily stressed in terms of power drawn from the controller. 

The direct interfaces between the controller and other subsystems are as follows:

1. Serial communication input/output to the vision microcontroller. This will be a general digital I/O pin. 
2. An input from the power system which triggers the microcontroller to start initializing code. This will be a digital input pin with hardware interrupt capabilities. 
3. Serial communication as an input to the object storage subsystem. This will be a digital output pin in terms of the microcontroller. 
4. Serial communication as an input to the object consumption subsystem. This will be a digital output pin in terms of the microcontroller.
5. The power input to the microcontroller, which will come from the power regulator in the power subsystem.  

Standard: IEEE 1118.1-1990 describes standards related to interdevicehtrabuilding as well as interconnection of microcontrollers. This stardard will be reference and adhered to in terms of system controller interconnect design, which is part of the controller subsystem. 

Ethics: Consideration will be taken for other people design which are contoller design will be specifically analyzed for safety in terms of automation. 

Socioeconomic: Economic consideration will need to be made when selecting a microcontroller, because microcontrollers can be very expensive to purchase.

Conceptual Design Document: [here](https://github.com/nathan-gardner/CapstoneRepo/blob/main/Reports/Team2_ConceptualDesignandPlanningFinal.pdf)

## Electronic Schematic 



## Analysis

Number of motors that need to be driven by the main controller and what purpose those motor serve are listed below

Feeding subsystem - two motor that need to be driven by the main microcontroller

Fireworks subsystem - zero or one motor to flip the switch for the fireworks

Locomotion subsystem - four motors driven independantly

Object Sorting and Storage - belt motor and motor which will poke the objects off of the belt



## BOM
