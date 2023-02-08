# Pedestal Storage and Delivery

## Function of the Subsystem
Throughout each round of the competition, the robot will need to intake up to seven pedestals. Once the pedestals are collected, they’ll need to be sorted and stored until it is the appropriate time to drop them off. The team plans to collect three pedestals before placing them wherever the robot may be upon the collection of the third pedestal. After that, it will place one or two stacks of two depending on whether or not all pedestals were collected. 
This subsystem’s main function is to ensure that the pedestals have a place to be stored as well as a method of delivery. The pedestals will be in the correct orientation from the sorting subsystem, and the silo in which each pedestal will stay will include two proximity sensors in order to detect the number of pedestals in the silo at all times. As previously stated, once three pedestals are detected in the robot, they will be dropped off by opening the silo and driving the robot away from the statue. For the next two iterations, the silo will open when there are only two pedestals since there are only seven pedestals total. The complexity of the subsystem is decreased as opposed to the duck delivery because the pedestals can be placed anywhere on the playing field. 
- Store all collected pedestals in the silos in the correct orientation	
- “Full” sensor will detect when there are two or three pedestals present 
- Drop the statues off whenever they are collected (first a statue that is three pedestals tall and after that, a statue that is two pedestals tall)
- Worst case scenario of three pedestals at a time is analyzed below

## Constraints
- The 1’x1’x1’ size constraint for the robot causes the design to need to be as area-effective as possible. There will only be one silo that is 5.5 in (height) x 2.25 in (width) x 2.25 inches (length).

- There will be two proximity sensors placed in the silo to send a signal back to the controller to interrupt the main course and drop off the pedestals. 

- The servo motor selected for the silo extension and pedestal drop off should be able to withstand the weight of at most three pedestals and the weight of the silo itself. In the worst case, the motor will have 0.0618 kg (the weight of three pedestals) + 0.06287 kg (the weight of the silo). It will need at least 3.495 $kg \ast cm$ of torque to function.

- Calculations for the rotation angle of the servo motor are in the analysis section below. The silo will need to open widde enough to let the statue fit through which will require .318 rotations or 115 degrees. The silo will remain open until enough time has elapsed that it can close again without trapping the statue within the silo.

- Another constraint that needs to be considered for this subsystem is the silo placement. The team’s plan is to drop off the pedestals upon collection. The silo will be on the side of the robot. The door will open and the robot will drive directly sideways away from the statue, leaving it behind. 

## Electrical Schematic
The electrical schematic is shown below.

![image](https://user-images.githubusercontent.com/112424739/217134565-60b39f12-6ada-46cd-a27a-911821b66bd0.png)

## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/216230141-b51adfd5-e3c0-481c-bdd2-9584e419ac03.png)
![image](https://user-images.githubusercontent.com/112424739/216230166-2ee72d46-860f-45da-af1e-fa78c745370b.png)
![image](https://user-images.githubusercontent.com/112424739/216230204-73e37f0f-681a-40a1-8120-c79857bd24f4.png)

Below is the rough layout of the chassis showing the major subsystems and their placement within the robot. This subsystem is in the block labeled "pedestals".

![chassis](https://user-images.githubusercontent.com/112424739/216861688-57e85d61-c0b9-467b-85f4-2b73cdc1b0f9.png)

Below are images showing how all of the subsystems, including the Pedestal Storage and Delivery subsystem will fit into the robot.

Top view

![image](https://user-images.githubusercontent.com/30758520/217405647-4aef4118-8f63-4c85-bbfe-5125365fd0a0.png)

Rear Side view 1

![image](https://user-images.githubusercontent.com/30758520/217406171-b0454923-5b69-47fe-aef1-dd19d3acfe9c.png)

Rear Side view 2

![image](https://user-images.githubusercontent.com/30758520/217406076-af1fa457-ba10-4bc0-bd10-c326b97fa633.png)

Rear view

![image](https://user-images.githubusercontent.com/30758520/217406232-0fdc2a19-cdb0-42c5-bda1-3b213afa7b6e.png)

Front corner

![image](https://user-images.githubusercontent.com/30758520/217406258-9ac6acc2-2f57-4e2d-a900-f2ad3f18a0f0.png)

Chassis Subsystem Fit CAD model files available [here](https://github.com/nathan-gardner/CapstoneRepo/tree/main/Documentation/3D%20Models/Chassis/Chassis_Fit).

## Analysis
### Size Calculations for Pedestal Silo

$Pedestal\ Volume = \pi \ast r^{2} \ast h = \pi \ast 1^{2} \ast 1.8 = 5.65 in^{3}$

$Pedestal\ Volume_{Total} = 5.65 \ast 3 = 16.96 in^{3}$

$Silo\ Volume = 5.5 \ast 2.25 \ast 2.25 = 27.84 in^{3}$

Silo Volume > Pedestal Volume. Therefore, the silos can hold the pedestals.

### Distance From Sensors

$P_{silo} = 2.25 \ast 4 = 9 in^{2}$

$C_{pedestal} = \pi \ast d = \pi \ast 2 = 6.28 in^{2}$

$D_{max} = C_{silo} - C_{pedestal} = 9 - 6.28 = 2.72 in = 0.0691 m$

The max distance calculated is less than the maximum distance the sensor can detect which is 5 meters. Therefore, the sensors will be able to detect the pedestals when they enter the silo.

### Weight Calculations

$W_{pedestals} = 20.6 g \ast 3 = 0.0618 kg$

$W_{silo} = 5.5 \ast 2.5 \ast 0.25 \ast% density of TPU = $3.4375 \ast 0.0448 = 0.154 lbs = 0.06287 kg$

$W_{total} = 0.0618 + 0.06287 = 0.12467 kg$

### Torque Calculations

$F = m \ast a = 0.12467 \ast 9.81 = 1.223 N$

$\tau = F \ast r \ast sin(\theta) = 1.223 \ast 0.07258 m \ast sin(90) = 0.0888 N \ast m = 0.9055$

The servo motor chosen is able to supply 3.5 $kg \ast cm$ of torque. Therefore, the servo will be sufficient for the design.

### Rotation Calculation for Statue Delivery

$C = \pi \ast d = \pi \ast 2 = 6.28$

$Rotations = \frac{ 2 }{ 12.56 } = 0.318$

$Degrees = rotations \ast 360 = 0.318 \ast 360 = 114.64&deg;$

This rotation angle is well within the servo's capabilities. 

### How Long will the Silo need to remain open?

To determine how long the silo will need to remain open to allow the statues to be left untouched, the speed of the robot must be determined. According to the locomotion subsystem, the robot will never be going slower than 0.0677 m/s when traversing the playing field. 

$0.0677 \frac{m}{s} = 2.665 \frac{in}{s}$

If the pedestal is two inches in diameter, the silo will need to remain open for no less than one second to allow the statue to exit the robot and remain intact and unharmed by the edges of the silo.

## BOM
| Name of Item           | Description                                  | Used in which subsystem(s)                | Part Number | Manufacturer     | Quantity | Price      | Total |
|------------------------|----------------------------------------------|-------------------------------------------|-------------|------------------|----------|------------|-------|
| Proximity Sensor       | Pololu Digital Sensor 5cm                    | Pedestal Storage and Delivery             | 4050        | Pololu           | 2        | 12.95      | 25.9  |
| Servo Motor            | FEEFETCH Standard Servo FT1117M                  | Pedestal Storage and Delivery             | 3423        | Pololu           | 1        | 9.95       | 9.95  |
| Silo                   | Silo to hold pedestals, 3D Printed           | Pedestal Storage and Delivery             | N/A         | N/A              | 1        | 0          | 0     |
| Servo Mount            | 3D Printed                                   | Pedestal Storage and Delivery             | N/A         | N/A              | 1        | 0          | 0     |
| Servo Motor Controller | Micro Maestro 6-channel USB Servo Controller | Duck Storage, Pedestal Storage, and Delivery and Feeding | 1350        | Pololu           | 1        | 39.95      | 39.95 |
| Total                  |                                              |                                           |             | Total Components | 6        | Total Cost | 75.8  |

