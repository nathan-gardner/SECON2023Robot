# Fireworks Subsystem

## Function of the Subsystem

The function of this subsystem is firework activation, which will need to be accomplished in the final seconds of the round to set off the animated fireworks display, which will signify that the round has ended. The team's plan is to accomplish this by running the robot into the switch on the far right hand side of the arena.

## Constraints

- The ability to provide enough force to flip the fireworks switch on collision with the switch (0.98 N)
- Natural chassis protrusion which will make contact with the switch

## Buildable Schematics

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/main/Documentation/Images/FireWorks/ChassiFullView.png)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/main/Documentation/Images/FireWorks/ChassisTopView.png)

Below are images showing how all of the subsystems will fit into the robot.

Top view

![image](https://user-images.githubusercontent.com/30758520/217405647-4aef4118-8f63-4c85-bbfe-5125365fd0a0.png)

Rear view

![image](https://user-images.githubusercontent.com/30758520/217406232-0fdc2a19-cdb0-42c5-bda1-3b213afa7b6e.png)

Rear Side view

![image](https://user-images.githubusercontent.com/30758520/217406171-b0454923-5b69-47fe-aef1-dd19d3acfe9c.png)

## Electrical Schematic

*N/A - no additional electrical design for this sign off*

## Analysis

Force analysis is included to show that the motors chosen in the locomotion subsystem can provide the torque needed to move the bot to activate a common house switch. 

The switch we are using is smaller than the lower estimation here.

Mass of the switch is close to 0.1 kg, acceleration is 9.8 m/s due to gravity

$F = m \ast a = 0.1 \ast 9.8 = 0.98\ N$

Since torque equals force times the moment arm length, so the torque would equal 

$0.98\ N \ast r_{wheel} = 0.98\ N \ast 0.024\ m = 0.0235\ N \ast m = 2.4\ kg \ast mm$ 

The required torque for a single motor in the drive train is 6.149 $kg \ast mm$, which is greater than the required $2.4\ kg \ast mm$, and we will have four of these motors. This means that the torque requirement within the drivetrain for flipping the switch is met. 

In terms of where the robot will catch the switch, the picture below shows the chassis assembly file, and shows the height of the corners of the robot (127 mm or 5 inches). This is higher than the switch will be off the ground and shows that the robot has four clean corner that will be able to catch the switch to activate it. The switch will be one and seven-eighths inches off the ground (according to the schematic provided by the competition) so the switch can catch on a sharp side of the robot to activate the switch.

![image](https://user-images.githubusercontent.com/30758520/217141747-341ffa6a-f6dc-4641-96af-541424a470a6.png)

Image showing the switch will be 1 and 7/8" off the ground is below

![image](https://user-images.githubusercontent.com/30758520/217413456-c2e14cfa-a8a8-4f76-acf1-6b79b0793c57.png)

## BOM

*N/A - no purchasing needed for this sign off*
