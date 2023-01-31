# Duck Storage and Delivery Subsystem Signoff

## Function of the Subsystem

Throughout each round of the competition, the robot will need to intake up to 17 items, 10 of which are ducks, via the consumption subsystem. These ducks that are collected will need to be sorted and stored until they are dropped off at their proper location on the playing board. The storage subsystem’s main function is to take in the ducks from the sorting subsystem, store them in their respective location, and drop them off at their proper location on the playing board. The ducks will be held in a corral connected to the back of the robot. When the predetermined path is completed, the robot will go to the duck pond and eject the corral via a rack and pinion mechanism.

### Function:

- Store the ducks in a bottomless corral that rolls behind the robot
- Have a locking mechanism that will ensure the corral is not lost when being rolled behind the robot
- Rack and pinion mechanism for rolling back the corral at the beginning of the competition as well as when the ducks are to be delivered.
- Drop the ducks and corral off in the duck pond via a drawer and lock-style solenoid mechanism
- Worst case scenario of all ten ducks at a time is analyzed below


## Constraints

- The first constraint for this subsystem is the space available within the robot. Like many other subsystems included in this project, the 1’x1’x1’ size constraint for the robot causes the design to need to be as area-effective as possible. To abide by this constraint, the team plans to have the ducks held in a bottomless corral that is outside of the robot. The corral, as previously mentioned, will surround part of the robot before the competition, and then as soon as the robot starts its path, it will extend from the back of the robot via a rack and pinion mechanism. 
- The ducks are approximately 3 inches x 3 inches x 3.5 inches, so the corral will need to be a sufficient size to hold all ten ducks. The ducks can sit on top of each other, so the corral shown in the buildable schematic will be sufficient to hold all ten ducks.
- The servo motor selected for the corral design should be able to push the weight of the ducks and the corral itself when ejecting the corral at the duck pond location. At worst case, the corral will be carrying all ten ducks, so the weight will be 0.708 kg (the weight of all ten ducks) + 1.04 kg (the weight of the corral). The friction coefficient of rubber on steel is approximately 0.76. This will need to be taken into consideration in the analysis for the rack and pinion.
- Calculations for the rotation angle of the servo motor are in the analysis section below. The distance that needs to be travelled for the initial roll back of the corral at the beginning of each round is 9 inches which will require 9 rotations or 3240&deg;. For the final drop off, it will need to travel 1 inch which will require 1 rotation or 360&deg;.
- The duck trailer will need to have a locking mechanism to ensure that it does not roll off of the rack when the robot is in motion. To address this, the team plans to add two lock-style solenoids and lock the trailer in place to stop any unwanted motion. They will insert into the grooves on the rack casing in order to relieve the servo from needing a certain back-driving torque.
- The pinion gear will need to have some way to "mesh" with the the rack that is mounted on the side of the trailer. In order to prevent it from coming off the rack, the team will 3D print an enclosure to ensure the pinion gear stays on the rack. The 3D model below shows this feature in more detail.
- Next, the omni-wheel on the back of the trailer should be attached such that the gap between the bottom of the back wall of the trailer and the playing field is not large enough that a duck or part of a duck could get jammed in the space and cause extra resistance on the robot's locomotion subsystem. To ensure that this is not an issue, the wheel will be adjustable so that the team can change the placement of the wheel to allow as much or as little of a gap as possible. The omni-wheel will also allow the trailer to follow any motion that the main body of the robot performs as well as lower the overall friction on the locomotion subsystem.
- This system has been taken into consideration for the locomotion subsystem in terms of overall weight and friction added to the motors for the locomotion subsystem. 
- The final constraint comes from the ethical consideration of a pinching hazard near the drawer and lock-style solenoid system. We will design the system so that the drawer slides are not directly exposed to the open, which would create a pinching hazard. This will create a safe environment for the team when they are working with the robot and will significantly reduce the chance of finger pinching near the system.

## Electrical Schematic

The electrical schematic for the object storage subsystem is shown below. The Servo Programmer comes with a battery pack that is able to power the servo. The voltage on the schematic is shown to come from the power subsystem as a worst case scenario in the event that this battery pack cannot be used.

![image](https://user-images.githubusercontent.com/112424739/215840819-dd22b409-ba04-4ebf-be2f-7d7cf625d118.png)

The link to the Schematic Document is found ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Duck-Storage-Delivery/Documentation/Electrical/Schematics/Sources).

## Analysis

### Torque Ratio Calculations

To find the Torque needed for the Servo Motor for the Rack and Pinion the following calculations were completed. Using 90&deg; as a worst case scenario.

$Total\ Mass = m_{ducks} + m_{trailer} = 0.708 + 1.04 = 1.748$

$F = m \ast 9.81 \ast 0.76 = 1.748 \ast 9.81 \ast 0.76 = 13.03 N$

$\tau = F \ast r \ast sin(\theta) = 13.03 \ast 0.0127 \ast sin(90&deg;) = 1.65 N \ast m$

$\tau = 1.65 N \ast m = 16.83 kg \ast cm$

Assuming a worst case velocity of spinning out the entire corral in 4 seconds, the velocity would need to be $2.5 \frac{in}{sec}$

$\omega = \frac{v}{r} = \frac{2.5}{05} = 5 \frac{rad}{sec} = 47.75 rpm$

We know that the servo motor that was selected can supply more than enough rpm, so it should be able to spin the corral out at the speed necessary.

### Rotation Calculation for Corral 

#### Initial Roll Back

$C = \pi \ast d = \pi \ast 1$

$Rotations = \frac{ distance }{ C } = \frac{ 9 }{ 1 } = 9$

$Degrees = rotations \ast 360 = 9 \ast 360 = 3240 &deg;$

#### Final Delivery

$C = \pi \ast d = \pi \ast 1$

$Rotations = \frac{ distance }{ C } = \frac{ 1 }{ 1 } = 1$

$Degrees = rotations \ast 360 = 1 \ast 360 = 360 &deg;$

### Lock-Style Solenoid Analysis

The lock is normally active, so it will not require power while in the locked state. It is designed for a 1-10 second activation time.

Keeping in mind that the duck trailer can be fully ejected in 4 seconds, this solenoid will be more than adequate for allowing the trailer to roll past when necessary.

### Size Calculations for Duck Corral

#### Duck Trailer
$Duck\ Volume = 3 \ast 3 \ast 3.5 = 31.5 in^{3}$

$Duck\ Volume_{Total} = 31.5 \ast 10 = 315 in^{3}$

$Corral\ Volume = 6 \ast 11.25 \ 9 = 607.5 in^{3}$
  
Corral Volume > Duck Volume. Therefore, the corral can hold the ducks.
  
#### Omni-Wheel Height
 
The onmi-wheel is adjustable, so the team can decide how far or how close to the ground the trailer should be. The duck's beak is approximately 0.25 inches.

In order to ensure the duck's beak does not get caught under the trailer, the omni-wheel must be placed at a height such that the trailer sides are no more than 0.25 inches off the ground. Due to the adjustable nature of the omni-wheel, this will be possible.

### Affect on Navigation

While the extra weight and friction was analyzed in the locomotion subsystem, the actual impact this will have on navigation will be analyzed within this one.

The navigation path will have to account for a robot with a length of 18 inches rather than 12 inches which would be the case without the trailer. In order to be as conservative as possible, we planned for the maximum robot length for our path that the robot will take. The playing field is 4 feet by 8 feet, there will be no objects closer than 2 inches to the walls, and our robot is now 18 inches long. This constrains our path when making turns. The consumption subsystem which is mounted to the front of the robot must also be facing the front when traversing the playing field. While making turns will be more difficult with the trailer, it will still be possible. Images of the path are shown below.

![image](https://user-images.githubusercontent.com/112424739/215837783-769432d2-edb6-4340-850b-d9f13149374c.png)
![image](https://user-images.githubusercontent.com/112424739/215837869-2f0f0a95-e123-46ca-a8d4-05c27026ee88.png)
![image](https://user-images.githubusercontent.com/112424739/215838003-47506fff-4d2e-4bee-bc78-4fd530f7a1cf.png)
![image](https://user-images.githubusercontent.com/112424739/215838073-c60859e9-c9a4-4c99-aa05-0559394c25fa.png)
![image](https://user-images.githubusercontent.com/112424739/215838162-e0a84f93-2456-4697-9389-b11788e35b7d.png)
![image](https://user-images.githubusercontent.com/112424739/215838232-433d4e69-bd52-4dc7-bce6-313151af797f.png)
![image](https://user-images.githubusercontent.com/112424739/215838621-543c3ad0-fb7f-4eba-b88a-9f10c96507da.png)
![image](https://user-images.githubusercontent.com/112424739/215838345-9452c8af-b4da-42a3-be39-4f687290fe67.png)
![image](https://user-images.githubusercontent.com/112424739/215838409-c32e8544-934d-474c-860a-23be8257859c.png)

Notice that the robot never goes more than 37.75 inches from the side wall. Therefore, this path will be the one the robot takes for each turn. 

### Timing

The affect this subsystem has on navigation will also affect the amount of time the competition rounds will take. This should not be a problem as the locomotion signoff calculates an 84.66\% margin of safety. This means that the motors only are using 15.34\% of their torque capacity, so there will be plenty of extra speed as needed.

## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/215122150-cb734a4b-a06e-434a-a5ee-3b39f7a0077a.png)

![image](https://user-images.githubusercontent.com/112424739/215122183-31111a78-5942-43d2-8b89-3af7396dcdd6.png)

![image](https://user-images.githubusercontent.com/112424739/215122204-f9507161-e863-45a5-899b-eb9bcbba79b0.png)

![image](https://user-images.githubusercontent.com/112424739/215122237-d9839cbe-5e73-428a-8454-a86f7dd8787a.png)

You can find the 3D models for all Components ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Duck-Storage-Delivery/Documentation/3D%20Models).

## BOM

| Name of Item           | Description                                                                                               | Used in which subsystem(s) | Part Number    | Manufacturer     | Quantity | Price      | Total  |
|------------------------|-----------------------------------------------------------------------------------------------------------|----------------------------|----------------|------------------|----------|------------|--------|
| Solenoid Mount         | Print                                                                                                     | Duck Storage               | N/A            | N/A              | 1        | 0          | 0      |
| Corner Bracket         | Print                                                                                                     | Duck Storage               | N/A            | N/A              | 4        | 0          | 0      |
| Steel Ball Transfer    | Steel Ball Transfer(21 mm Height)                                                                         | Duck Storage               | 1619-001-001   | Servo City       | 1        | 2.99       | 2.99   |
| Lock-Style Solenoids   | 12 V-DC                                                                                                   | Duck Storage               | 1512           | Adafruit         | 2        | 14.95      | 29.9   |
| Aluminum Sheet         | Sides and Backing for Duck Corral - Multipurpose 6061 Aluminum Sheet, 1/8'' thick, 6'' x 12''             | Duck Storage               | 6061           | McMaster Carr    | 3        | 24.28      | 72.84  |
| Rack                   | 14-1/2 Degree Pressure Angle Gear Rack, 32 Pitch (1ft) Nylon Plastic                                      | Duck Storage               | 57655K62       | McMaster Carr    | 2        | 8.94       | 17.88  |
| Servo-Mounted Pinion   | 32P, 32 Tooth, 25T 3F Spline Servo Mount Gear (Acetyl)                                                    | Duck Storage               | RSA32-2FS-32   | McMaster Carr    | 2        | 3.94       | 7.88   |
| Servo Motors           | 2000 Series Dual Mode Servo                                                                               | Duck Storage               | 2000-0025-0002 | Servo City       | 2        | 31.99      | 63.98  |
| Servo Motor Controller | 3102 Series Dual Mode Servo Programmer                                                                    | Duck Storage               | 3102-0001-0001 | Servo City       | 2        | 9.99       | 19.98  |
| TIP102 Transistors     | Darlington Transistors in order to have the ability to switch on and off the solenoid to lock the trailer | Duck Storage               | 976            | Adafruit         | 1        | 2.5        | 2.5    |
| Total                  |                                                                                                           |                            |                | Total Components | 20       | Total Cost | 217.95 |



