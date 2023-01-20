# Pedestal Storage and Delivery

## Function of the Subsystem
Throughout each round of the competition, the robot will need to intake up to seven pedestals. Once the pedestals are collected, they’ll need to be sorted and stored until it is the appropriate time to drop them off. The team plans to collect three pedestals before placing them wherever the robot may be upon the collection of the third pedestal. After that, it will place one or two stacks of two depending on whether or not all pedestals were collected. 
This subsystem’s main function is to ensure that the pedestals have a place to be stored as well as a method of delivery. The pedestals will be in the correct orientation from the sorting subsystem, and the silo in which each pedestal will stay will include two proximity sensors in order to detect the number of pedestals in the silo at all times. As previously stated, once three pedestals are detected in the robot, they will be dropped off by opening the silo and driving the robot away from the statue. For the next two iterations, the silo will open when there are only two pedestals since there are only seven pedestals total. The complexity of the subsystem is decreased as opposed to the duck delivery because the pedestals can be placed anywhere on the playing field. 
- Store all collected pedestals in the silos in the correct orientation	
- “Full” sensor will detect when there are two or three pedestals present 
- Drop the statues off whenever they are collected (first a statue that is three pedestals tall and after that, a statue that is two pedestals tall)
- Worst case scenario of three pedestals at a time is analyzed below

## Constraints
The first constraint for this subsystem is the space available within the robot. Like many other subsystems included in this project, the 1’x1’x1’ size constraint for the robot causes the design to need to be as area-effective as possible. To abide by this constraint, the size and number of silos will need to be considered. There will only be one silo. In order to save as much space as possible, the silo will only be tall enough to store three pedestals at once.  

Because the robot contains only one silo that can hold at most three pedestals, there needs to be a way to detect if there are three pedestals present. The plan is to drop off the first three collected and then two sets of two after that, so there also needs to be a way to detect if there are three pedestals present in the silo. To abide by these constraints, two proximity sensors will be placed in the silo to send a signal back to the controller to interrupt the main course and drop off the pedestal statues. 

The servo motor selected for the silo extension and pedestal drop off should be able to withstand the weight of at most three pedestals and the weight of the silo itself. In the worst case, the motor will have 0.0618 kg (the weight of three pedestals) + 0.1 kg (the weight of the silo).

This motor will need to be controlled in order to ensure that it opens wide enough to leave the statue standing and not keep it within the silo. The use of a servo-specific motor controller will aid in the control of the servos. As well as this, calculations for the rotation angle of the servo motor are in the analysis section below. The silo will need to open 2 inches which will require 1 rotation or 360 degrees. The silo will remain open until enough time has elapsed that it can close again without trapping the statue within the silo.

Another constraint that needs to be considered for this subsystem is the silo placement. The team’s plan is to drop off the pedestals upon collection. This decision was made based on point value versus difficulty level of finding the inner circles to place the statues. Thus, the robot could be anywhere within the playing field when there are two or three pedestals collected for drop-off, so the silo needs to be placed in a strategic location. The silo will be placed at the back corner of the robot so that it opens towards the outside of the robot. Since the duck trailer is also placed at the back of the robot, it will need to be as close to one side as possible. Placing the pedestal silo at the back corner will allow for an easy drop-off. The silo will open and the robot will drive away, leaving the statue behind. After the robot drives away, the silo will close and the process will repeat up to two more times during the competition round.

## Electrical Schematic

![image](https://user-images.githubusercontent.com/112424739/213763728-cefd6d52-0207-4277-8a8a-7375e14ff809.png)


## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/213576212-9a67606e-6bfd-4f47-9b77-9e765276380f.png)

![image](https://user-images.githubusercontent.com/112424739/213576274-2bf06847-a65c-4937-8941-d07af3cfdbbf.png)

![image](https://user-images.githubusercontent.com/112424739/213576352-f804ddae-cf98-4ab0-96b0-ac3213059855.png)


## Analysis
### Size Calculations for Pedestal Silo

$Pedestal\ Volume = \pi \ast r^{2} \ast h = \pi \ast 1^{2} \ast 1.8 = 5.65 in^{3}$

$Pedestal\ Volume_{Total} = 5.65 \ast 3 = 16.96 in^{3}$

$Silo\ Volume = \pi \ast r^{2} \ast h = \pi \ast 1.083^{2} \ast 6 = 22.108 in^{3}$

Silo Volume > Pedestal Volume. Therefore, the silos can hold the pedestals.

### Rotation Calculation for Statue Delivery

$C = \pi \ast d = \pi \ast 2$

$Rotations = \frac{ 2 }{ 2 } = \frac{ 2 }{ 2 }$

$Degrees = rotations \ast 360 = 1 \ast 360 = 360&deg;$

### Distance From Sensors

$C_{silo} = \pi \ast d = \pi \ast 2.165 in^{2} = 6.80 in^{2}$

$C_{pedestal} = \pi \ast d = \pi \ast 2 = 6.28 in^{2}$

$D_{max} = C_{silo} - C_{pedestal} = 6.80 - 6.28 = 0.52 in = 0.0133 m$

The max distance calculated is less than the maximum distance the sensor can detect which is 5 meters. Therefore, the sensors will be able to detect the pedestals when they enter the silo.

### How Long will the Silo need to remain open?

To determine how long the silo will need to remain open to allow the statues to be left untouched, the speed of the robot must be determined. According to the locomotion subsystem, the robot will never be going slower than 0.0677 m/s when traversing the playing field. 

$0.0677 \frac{m}{s} = 2.665 \frac{in}{s}$

If the pedestal is two inches in diameter, the silo will need to remain open for no less than one second to allow the statue to exit the robot and remain intact and unharmed by the edges of the silo.

## BOM

