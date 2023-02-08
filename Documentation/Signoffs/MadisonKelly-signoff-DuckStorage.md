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

- To abide by the 1 cubic foot size constraint, the trailer will extend from the back of the robot via a rack and pinion mechanism. 
- The ducks are approximately 3 inches x 3 inches x 3.5 inches, so the corral will need to be a sufficient size to hold all ten ducks. The ducks can sit on top of each other, so the corral shown in the buildable schematic will be sufficient to hold all ten ducks.
- The servo motor selected for the corral design should have at least 16.83 $kg \ast cm$ of torque.
- The distance that needs to be travelled for the initial roll back of the corral at the beginning of each round is 9 inches which will require 9 rotations or 3240&deg;. For the final drop off, it will need to travel 1 inch which will require 1 rotation or 360&deg;.
- The duck trailer will need to have a locking mechanism to ensure that it does not roll off of the rack when the robot is in motion. To address this, the team plans to add two lock-style solenoids and lock the trailer in place to stop any unwanted motion. 
- The pinion gear will need to have some way to "mesh" with the the rack that is mounted on the side of the trailer. In order to prevent it from coming off the rack, the team will 3D print an enclosure to ensure the pinion gear stays on the rack. The 3D model below shows this feature in more detail.
- The omni-wheel on the back of the trailer should be attached such that the gap between the bottom of the back wall of the trailer and the playing field is not large enough that a duck or part of a duck could get jammed in the space and cause extra resistance on the robot's locomotion subsystem.
- This system will affect the path taken along the playing field as well as the time constraints. 
- The final constraint comes from the ethical consideration of a pinching hazard near the drawer and lock-style solenoid system. We will design the system so that the drawer slides are not directly exposed to the open, which would create a pinching hazard. 

## Electrical Schematic

The electrical schematic for the object storage subsystem is shown below. Please note that the circuit components will be implemented via a solderable breadboard. The circuit layout is as specified by the manufacturer's datasheet.

![image](https://user-images.githubusercontent.com/112424739/216155292-916236a9-e371-4c9d-9cbd-1963f927a87e.png)

The link to the Schematic Document is found ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Duck-Storage-Delivery/Documentation/Electrical/Schematics/Sources).

## Analysis

### Size Calculations for Duck Corral

#### Duck Trailer
$Duck\ Volume = 3 \ast 3 \ast 3.5 = 31.5 in^{3}$

$Duck\ Volume_{Total} = 31.5 \ast 10 = 315 in^{3}$

$Corral\ Volume = 6 \ast 11.25 \ 9 = 607.5 in^{3}$
  
Corral Volume > Duck Volume. Therefore, the corral can hold the ducks.

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

Notice that the robot never goes more than 37.75 inches from the side wall. Therefore, this path will be the one the robot takes for each turn. According to the pre-determined path, the robot will only have to make four turns when doing the initial traversal of the playing field. 

### Timing

The affect this subsystem has on navigation will also affect the amount of time the competition rounds will take. According to the pre-determined path, the robot will only have to make four turns when doing the initial traversal of the playing field. This turning mechanism will add an additional 34.5 inches to our path per turn. 

$4\ turns \ast 34.5\ in = 138\ inches$

$480\ total\ path\ length + 138\ inches = 618\ in$

According to the locomotion subsystem's calculations, the robot can travel at up to 0.2023 $\frac{m}{s}$. Assuming the robot can go this full speed on straight-aways, the following calculations are performed.

$480\ in = 12.192\ m = \frac{12.192}{0.2023} = 60.3\ sec$

Assuming the turns will take 5 seconds each,

$60.3\ sec + 4 \ast 5\ sec = 80.3\ sec$

Therefore, the robot can still traverse the entire playing field in less than half the time (3 minutes). This leaves an additional minute and 40 seconds to complete all the other tasks. In addition to this, locomotion signoff calculates an 84.66\% margin of safety. This means that the motors only are using 15.34\% of their torque capacity, so there will be plenty of extra speed as needed.

## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/215122150-cb734a4b-a06e-434a-a5ee-3b39f7a0077a.png)

![image](https://user-images.githubusercontent.com/112424739/215122183-31111a78-5942-43d2-8b89-3af7396dcdd6.png)

![image](https://user-images.githubusercontent.com/112424739/215122204-f9507161-e863-45a5-899b-eb9bcbba79b0.png)

![image](https://user-images.githubusercontent.com/112424739/215122237-d9839cbe-5e73-428a-8454-a86f7dd8787a.png)

You can find the 3D models for all Components ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Duck-Storage-Delivery/Documentation/3D%20Models).

Below are images showing how all of the subsystems, including Duck Storage Subsystem will fit into the robot.

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

## BOM

| Name of Item           | Description                                                                                               | Used in which subsystem(s)                  | Part Number    | Manufacturer     | Quantity | Price      | Total |
|------------------------|-----------------------------------------------------------------------------------------------------------|---------------------------------------------|----------------|------------------|----------|------------|-------|
| Solenoid Mount         | Print                                                                                                     | Duck Storage                                | N/A            | N/A              | 1        | 0          | 0     |
| Corner Bracket         | Print                                                                                                     | Duck Storage                                | N/A            | N/A              | 4        | 0          | 0     |
| Steel Ball Transfer    | Steel Ball Transfer(21 mm Height)                                                                         | Duck Storage                                | 1619-001-001   | Servo City       | 1        | 2.99       | 2.99  |
| Lock-Style Solenoids   | 12 V-DC                                                                                                   | Duck Storage                                | 1512           | Adafruit         | 2        | 14.95      | 29.9  |
| Aluminum Sheet         | Sides and Backing for Duck Corral - Multipurpose 6061 Aluminum Sheet, 1/8'' thick, 6'' x 12''             | Duck Storage                                | 6061           | McMaster Carr    | 3        | 24.28      | 72.84 |
| Rack                   | 14-1/2 Degree Pressure Angle Gear Rack, 32 Pitch (1ft) Nylon Plastic                                      | Duck Storage                                | 57655K62       | McMaster Carr    | 2        | 8.94       | 17.88 |
| Servo-Mounted Pinion   | 32P, 32 Tooth, 25T 3F Spline Servo Mount Gear (Acetyl)                                                    | Duck Storage                                | RSA32-2FS-32   | McMaster Carr    | 2        | 3.94       | 7.88  |
| Servo Motors           | 2000 Series Dual Mode Servo                                                                               | Duck Storage                                | 2000-0025-0002 | Servo City       | 2        | 31.99      | 63.98 |
| Servo Motor Controller | 3102 Series Dual Mode Servo Programmer                                                                    | Duck Storage                                | 3102-0001-0001 | Servo City       | 2        | 9.99       | 19.98 |
| TIP102 Transistors     | Darlington Transistors in order to have the ability to switch on and off the solenoid to lock the trailer | Duck Storage                                | 976            | Adafruit         | 1        | 2.5        | 2.5   |
| Servo Controller       | Micro Maestro 6-Channel USB Servo Controller                                                              | Duck Storage, Pedestal Storage, and Feeding | 1350           | Pololu           | 1        | 39.95      | 39.95 |
| Total                  |                                                                                                           |                                             |                | Total Components | 21       | Total Cost | 257.9 |




