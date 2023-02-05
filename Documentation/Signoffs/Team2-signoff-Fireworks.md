# Fireworks Subsystem

## Function of the Subsystem

The function of this subsystem is firework activation, which will need to be accomplished in the final seconds of the round to set off the animated fireworks display. The team's plan is to accomplish this by running the robot into the switch on the far right hand side of the arena.

## Constraints

- The ability to provide enough force to flip the fireworks switch on collision with the switch
- Natural chassis protrusion which will make contact with the switch

## Buildable Schematics

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/Team2_Fireworks_Signoff/Documentation/Images/FireWorks/ChassiFullView.png)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/Team2_Fireworks_Signoff/Documentation/Images/FireWorks/ChassisTopView.png)

**Include a model of the chassis point which will make contact with the switch.**

## Electrical Schematic

*N/A - no additional electrical design for this sign off*

## Analysis

Force analysis is included to show that the motors chosen in the locomotion subsystem can provide the torque needed to move the bot to activate a common house switch. 

The switch we are using is smaller than the lower estimation here.

Mass of the switch is close to $0.1\ kg$, acceleration is $9.8\ m/s$ due to gravity
$F = m * a = 0.1 * 9.8 = 0.98\ N$

Since torque equals force times the moment arm length, so the torque would equal $0.98\ N * r_{wheel} = 0.98\ N * 0.024\ m = 0.0235\ N*m = 2.4\ kg*mm$ of torque. The required torque for a single motor in the drive train is 6.149 kg*mm and we will have four of these motors. This means that the torque requirement within the drivetrain for flipping the switch is met. 

## BOM

*N/A - no purchasing needed for this sign off*