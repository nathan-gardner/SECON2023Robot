# Duck Storage and Delivery Subsystem Signoff

## Function of the Subsystem

Throughout each round of the competition, the robot will need to intake up to 17 items, 10 of which are ducks, via the consumption subsystem. These ducks that are collected will need to be sorted and stored until they are dropped off at their proper location on the playing board. The storage subsystem’s main function is to take in the ducks from the sorting subsystem, store them in their respective location, and drop them off at their proper location on the playing board. The ducks will be held in a corral connected to the back of the robot. When the predetermined path is completed, the robot will go to the duck pond and eject the corral via a drawer slide and electromagnetic solenoid mechanism.

### Function:

- Store the ducks in a bottomless corral that rolls behind the robot
- Have a locking mechanism that will ensure the corral is not lost when being rolled behind the robot
- Drop the ducks and corral off in the duck pond via a drawer and electromagnetic solenoid mechanism
- Worst case scenario of all ten ducks at a time is analyzed below


## Constraints

The first constraint for this subsystem is the space available within the robot. Like many other subsystems included in this project, the 1’x1’x1’ size constraint for the robot causes the design to need to be as area-effective as possible. To abide by this constraint, the team plans to have the ducks held in a corral that is outside of the robot. The corral, as previously mentioned, will surround part of the robot before the competition, and then as soon as the robot starts its path, it will extend from the back of the robot via a drawer-like mechanism. The drawer slides will be mounted on an angle so gravity will work on the corral causing it to roll back when the locks are not in place. There will be two sets of solenoid locks on the drawer slides in order to prevent the corral from rolling back and off of the robot before it reaches its final destination. The ducks are approximately 3 inches x 3 inches x 3.5 inches, so the corral will need to be a sufficient size to hold all ten ducks. The ducks can sit on top of each other, so the corral shown in the buildable schematic will be sufficient to hold all ten ducks.

The corral will roll behind the robot for the entirety for the competition until the robot reaches its final destination at the duck pond. Once the robot reaches the duck pond, it will drop the corral off with all of the ducks inside and leave them behind by dropping the "locks" within the drawer slides which will eject the corral and ducks. The corral will be bottomless so the ducks can be counted as touching the duck pond when they are dropped off. This will save much needed space within the robot and save time when depositing the ducks at their duck pond location.

The duck trailer will need to have a locking mechanism to ensure that it does not roll off of the rack when the robot is in motion. To address this, the team plans to add four lock-style solenoids and lock the trailer in place to stop any unwanted motion.

The trailer for the ducks should be large enough to ensure all ten ducks fit within the volume of the trailer. The approximate volume of a duck is 3'x 3'x 3.5', so the trailer must be at least 315 cubic inches. 

The duck trailer should be bottomless so that at the end of each competition round when the ducks are delivered to the duck pond, the ducks will touch the surface of the pond and not just the bottom of the trailer. This constraint ensures that we will get the points rewarded for delivering all ducks present in the trailer to the duck pond. 

Next, the omni-wheel on the back of the trailer should be attached such that the gap between the bottom of the back wall of the trailer and the playing field is not large enough that a duck or part of a duck could get jammed in the space and cause extra resistance on the robot's locomotion subsystem. To ensure that this is not an issue, the wheel will be adjustable so that the team can change the placement of the wheel to allow as much or as little of a gap as possible. The omni-wheel will also allow the trailer to follow any motion that the main body of the robot performs.

This system is also constrained by the path that the robot must take to get to each destination. Throughout the course of the competition, the robot will have to navigate tight corners and travel in every and all directions. To mitigate this constraint, the trailer will have an omni-wheel on the back as well as have two connection points to the robot. This will allow the trailer to closely follow the robot and move in any direction it needs. Also, the size and shape of the trailer will be such that it can minimize its affect on the navigation system.

The final constraint comes from the ethical consideration of a pinching hazard near the drawer and lock-style solenoid system. We will design the system so that the drawer slides are not directly exposed to the open, which would create a pinching hazard. This will create a safe environment for the team when they are working with the robot and will significantly reduce the chance of finger pinching near the system.

## Electrical Schematic

The electrical schematic for the object storage subsystem is shown below.

![image](https://user-images.githubusercontent.com/112424739/213255596-c33bc34b-75c8-4e97-854c-119196ea2d17.png)

The link to the Schematic Document is found ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Storage/Documentation/Electrical/Schematics/Sources/ElectricalSchematicforStorage).

## Analysis

### Lock-Style Solenoid Analysis

The lock is normally active, so it will not require power while in the locked state. It is designed for a 1-10 second activation time.

Assuming the wheel on the drawer slide will need to move 1 inch and the slides are installed at a $20 &deg;,$

Trailer Acceleration $= g \ast sin(\theta) = 386.09 \ast sin(20) = 132.05 \frac{in}{sec^{2}}$

Since the lock can remain open for up to 10 seconds, this will be more than adequate for allowing the trailer to roll past when necessary.


### Size Calculations for Duck Corral

$Duck\ Volume = 3 \ast 3 \ast 3.5 = 31.5 in^{3}$

$Duck\ Volume_{Total} = 31.5 \ast 10 = 315 in^{3}$

$Corral\ Volume = 6 \ast 11.25 \ 9 = 607.5 in^{3}$
  
Corral Volume > Duck Volume. Therefore, the corral can hold the ducks.
  


## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/203194135-34b73e28-fe2f-450b-8afe-6c91816f216c.png)

![image](https://user-images.githubusercontent.com/112424739/203194286-09d109c4-49ef-47ed-850c-6da6ccd6d95f.png)

![image](https://user-images.githubusercontent.com/112424739/203194417-bcac0ef0-fdde-430a-8116-afd4ed4b782a.png)

![image](https://user-images.githubusercontent.com/112424739/203194486-e8f25d12-fddf-4722-b17f-729f1d32bdeb.png)

You can find the 3D models for all Components ![here](https://github.com/nathan-gardner/CapstoneRepo/tree/MadisonKelly-signoff-Storage/Documentation/3D%20Models).

## BOM

| Solenoid Mount       | Print                                                                                         | Duck Storage | N/A          | N/A              | 1  | 0          | 0      |
|----------------------|-----------------------------------------------------------------------------------------------|--------------|--------------|------------------|----|------------|--------|
| Corner Bracket       | Print                                                                                         | Duck Storage | N/A          | N/A              | 4  | 0          | 0      |
| Steel Ball Transfer  | Steel Ball Transfer(21 mm Height)                                                             | Duck Storage | 1619-001-001 | Servo City       | 1  | 2.99       | 2.99   |
| Lock-Style Solenoids | 12 V-DC                                                                                       | Duck Storage | 1512         | Adafruit         | 4  | 14.95      | 59.8   |
| Aluminum Sheet       | Sides and Backing for Duck Corral - Multipurpose 6061 Aluminum Sheet, 1/8'' thick, 6'' x 12'' | Duck Storage | 6061         | McMaster Carr    | 3  | 24.28      | 72.84  |
| Total                |                                                                                               |              |              | Total Components | 13 | Total Cost | 135.63 |


