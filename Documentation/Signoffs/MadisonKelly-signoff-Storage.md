# Storage Subsystem Signoff

## Function of the Subsystem

Throughout each round of the competition, the robot will need to intake up to 17 items via the consumption subsystem. These ducks and pedestals collected will need to be sorted and stored until they are dropped off at their proper locations on the playing board. The storage subsystem’s main function is to take in the ducks and pedestals from the sorting subsystem, store them in their respective locations, and drop them off at their proper location on the playing board. The ducks will be held in a corral connected to the back of the robot, and the pedestals will be kept in a silo in the proper orientation within the robot. There will be sensors in the silo to ensure no more than three pedestals are in the robot at one time, and when the sensor indicates three pedestals, the silo will extend out and drop off the pedestals before moving on. When the predetermined path is completed, the robot will go to the duck pond and eject the corral via a rack and pinion mechanism.

### Function:

- Store the ducks in a bottomless corral that drags behind the robot
- Have a locking mechanism that will ensure the corral is not lost when being dragged behind the robot
- Drop the ducks and corral off in the duck pond via a rack and pinion mechanism
- Worst case scenario of all ten ducks at a time is analyzed below


## Constraints

The first constraint for this subsystem is the space available within the robot. Like many other subsystems included in this project, the 1’x1’x1’ size constraint for the robot causes the design to need to be as area-effective as possible. To abide by this constraint, the team plans to have the ducks held in an outside corral. The corral, as previously mentioned, will surround part of the robot before the competition, and then as soon as the robot starts its path, it will extend from the back of the robot via a rack and pinion mechanism. The ducks are approximately 3 inches x 3 inches x 3.5 inches, so the corral will need to be a sufficient size to hold all ten ducks. They ducks can sit on top of each other, so the corral shown in the buildable schematic will be sufficient to hold all ten ducks.

The corral will roll behind the robot for the entirety for the competition until the robot reaches its final destination at the duck pond. Once the robot reaches the duck pond, it will drop the corral off with all of the ducks inside and leave them behind by spinning the gear from the rack and pinion which will eject the corral and ducks. The corral will be bottomless so the ducks can be counted as touching the duck pond when they are dropped off. This will save much needed space within the robot and save time when depositing the ducks at their duck pond location.

The servo motor selected for the corral design should be able to push the weight of the ducks and the corral itself when ejecting the corral at the duck pond location. At worst case, the corral will be carrying all ten ducks, so the weight will be 0.708 kg (the weight of all ten ducks) + 1.04 kg (the weight of the corral). The friction coefficient of rubber on steel is approximately 0.76. This will need to be taken into consideration in the analysis for the rack and pinion.

<<<<<<< Updated upstream
The servo motor selected for the silo extension and pedestal drop off should be able to withstand the weight of at most three pedestals and the weight of the silo itself. In the worst case, the motor will have 0.0618 kg (the weight of three pedestals) + 0.1 kg (the weight of the silo). For simplicity, the servo selected for the silo opening will be the same one as used in the rack and pinion mechanism. Since the weight of ten ducks plus the weight of the corral greatly outweighs three pedestals and the silo, the servo motor selected for the rack and pinion will have sufficient torque to open and close the silo.

Both of these motors will need to be controlled in order to determine how many rotations the motor needs to make to extend the corral and open the silo for duck and pedestal drop off, respectively. The angle that the servo motors need to rotate will need to be precisely measured so that the corral is not lost in the initial roll back and the pedestals are not trapped within the silo. The use of a servo-specific motor controller will aid in the control of the servos. As well as this, calculations for the rotation angle of the servo motor are in the analysis section below. The distance that needs to be traveled for the initial roll back of the corral at the beginning of each round is 9 inches which will require 9 rotations or 3240&deg;. For the final drop off, it will need to travel 1 inch which will require 1 rotation or 360&deg;. The silo will need to open 2 inches which will require 1 rotations or 360&deg;. The silo will remain open until enough time has elapsed that it can close again without trapping the statue was just placed.
=======
This motor will need to be controlled in order to determine how many rotations the motor needs to make to extend the corral for duck drop off. The angle that the servo motors need to rotate will need to be precisely measured so that the corral is not lost in the initial roll back. The use of a servo-specific motor controller will aid in the control of the servos. As well as this, calculations for the rotation angle of the servo motor are in the analysis section below. The distance that needs to be travelled for the initial roll back of the corral at the beginning of each round is 9 inches which will require 9 rotations or 3240&deg;. For the final drop off, it will need to travel 1 inch which will require 1 rotation or 360&deg;.
>>>>>>> Stashed changes

The final constraint come from the ethical consideration of a pinching hazard near the rack and pinion system. We will design the rack and pinion so that the gear is not directly exposed to the open, which would create a pinching hazard. This will create a safe environment for the team when they are working with the robot and will significantly reduce the chance of finger pinching near the gear for the rack and pinion system.

## Electrical Schematic

The electrical schematic for the object storage subsystem is shown below.

![image](https://user-images.githubusercontent.com/112424739/203210837-6efa8021-ff65-49b0-981e-cfd7684371c9.png)

The link to the Schematic Document is found ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Storage/Documentation/Electrical/Schematics/Sources/ElectricalSchematicforStorage).

## Analysis

### Torque Ratio Calculations

To find the Torque needed for the Servo Motor for the Rack and Pinion the following calculations were completed. Using 90&deg; as a worst case scenario.

$Total\ Mass = m_{ducks} + m_{trailer} = 0.708 + 1.04 = 1.748$

$F = m \ast 9.81 \ast 0.76 = 1.748 \ast 9.81 \ast 0.76 = 13.03 N$

$\tau = F \ast r \ast sin(\theta) = 13.03 \ast 0.0127 \ast sin(90) = 1.65 N \ast m$

$\tau = 1.65 N \ast m = 16.83 kg \ast cm$

Assuming a worst case velocity of spinning out the entire corral in 4 seconds, the velocity would need to be $2.5 \frac{in}{sec}$

$\omega = \frac{v}{r} = \frac{2.5}{05} = 5 \frac{rad}{sec} = 47.75 rpm$

We know that the servo motor that was selected can supply this amount of rpm, so it should be able to spin the corral out at the speed necessary.

### Size Calculations for Duck Corral

$Duck\ Volume = 3 \ast 3 \ast 3.5 = 31.5 in^{3}$

$Duck\ Volume_{Total} = 31.5 \ast 10 = 315 in^{3}$

$Corral\ Volume = 6 \ast 11.25 \ 9 = 607.5 in^{3}$
  
Corral Volume > Duck Volume. Therefore, the corral can hold the ducks.
  
### Rotation Calculation for Corral 

#### Initial Roll Back

$C = \pi \ast d = \pi \ast 1$

$Rotations = \frac{ distance }{ C } = \frac{ 9 }{ 1 } = 9$

$Degrees = rotations \ast 360 = 9 \ast 360 = 3240 &deg;$

#### Final Delivery

$C = \pi \ast d = \pi \ast 1$

$Rotations = \frac{ distance }{ C } = \frac{ 1 }{ 1 }$

$Degrees = rotations \ast 360 = 1 \ast 360 = 360 &deg;$

## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/203194135-34b73e28-fe2f-450b-8afe-6c91816f216c.png)

![image](https://user-images.githubusercontent.com/112424739/203194286-09d109c4-49ef-47ed-850c-6da6ccd6d95f.png)

![image](https://user-images.githubusercontent.com/112424739/203194417-bcac0ef0-fdde-430a-8116-afd4ed4b782a.png)

![image](https://user-images.githubusercontent.com/112424739/203194486-e8f25d12-fddf-4722-b17f-729f1d32bdeb.png)

You can find the 3D models for all Components ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Storage/Documentation/3D%20Models).

## BOM

| Name of Item           | Description                                                                                   | Used in which subsystem(s) | Part Number    | Manufacturer     | Quantity | Price      | Total  |
|------------------------|-----------------------------------------------------------------------------------------------|----------------------------|----------------|------------------|----------|------------|--------|
| Proximity Sensor       | Pololu Digital Sensor 5cm                                                                     | Object Storage             | 4050           | Pololu           | 2        | 12.95      | 25.9   |
| Servo Motor            | 2000 Series Dual-Mode Servo (25-2, Torque)                                                    | Object Storage             | 2000-0025-0002 | Servo City       | 2        | 23.99      | 47.98  |
| Servo Motor Controller | Micro Maestro 6-channel USB Servo Controller                                                  | Object Storage             | 1350           | Pololu           | 1        | 34.95      | 34.95  |
| Silo                   | Silo to hold pedestals                                                                        | Object Storage             |                |                  | 1        | 0          | 0      |
| Servo Mount            | Print                                                                                         | Object Storage             | N/A            | N/A              | 2        | 0          | 0      |
| Corner Bracket         | Print                                                                                         | Object Storage             | N/A            | N/A              | 4        | 0          | 0      |
| Rack                   | 14-1/2 Degree Pressure Angle Gear Rack, 32 Pitch (1ft) Nylon Plastic                          | Object Storage             | 57655K62       | McMaster Carr    | 2        | 8.94       | 17.88  |
| Servo-Mounted Pinions  | 32P, 32 Tooth, 25T 3F Spline Servo Mount Gear (Acetyl)                                        | Object Storage             | RSA32-2FS-32   | Servo City       | 2        | 3.94       | 7.88   |
| Steel Ball Transfer    | Steel Ball Transfer(21 mm Height)                                                             | Object Storage             | 1619-001-001   | Servo City       | 1        | 2.99       | 2.99   |
| Aluminum Sheet         | Sides and Backing for Duck Corral - Multipurpose 6061 Aluminum Sheet, 1/8'' thick, 6'' x 12'' | Object Storage             | 6061           | McMaster Carr    | 3        | 24.28      | 72.84  |
| Total                  |                                                                                               |                            |                | Total Components | 20       | Total Cost | 210.42 |

