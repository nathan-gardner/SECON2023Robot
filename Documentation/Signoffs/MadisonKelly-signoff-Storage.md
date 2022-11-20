# Storage Subsystem Signoff

## Function of the Subsystem

Throughout each round of the competition, the robot will need to intake up to 17 items via the consumption subsystem. These ducks and pedestals collected will need to be sorted and stored until they are dropped off at their proper locations on the playing board. The storage subsystem’s main function is to take in the ducks and pedestals from the sorting subsystem, store them in their respective locations, and drop them off at their proper location on the playing board. The ducks will be held in a “trailer” connected to the back of the robot, and the pedestals will be kept in a silo in the proper orientation within the robot. There will be sensors in the silo to ensure no more than three pedestals are in the robot at one time, and when the sensor indicates three pedestals, the silo will extend out and drop off the pedestals before moving on. When the predetermined path is completed, the robot will go to the duck pond and eject the trailer via a rack and pinion mechanism.

### Function:

- Store all collected pedestals in the silos in the correct orientation	
- “Full” sensor will detect when there are two or three pedestals present 
- Drop the statues off whenever they are collected (first a statue that is three pedestals tall and after that, a statue that is two pedestals tall)
- Store the ducks in a bottomless trailer that drags behind the robot
- Drop the ducks and trailer off in the duck pond via a rack and pinion mechanism
- Worst case scenario of all ten ducks and three pedestals at a time is analyzed below


## Constraints

The first constraint for this subsystem is the space available within the robot. Like many other subsystems included in this project, the 1’x1’x1’ size constraint for the robot causes the design to need to be as area-effective as possible. To abide by this constraint, the team plans to have the ducks held in an outside “trailer”. The trailer, as previously mentioned, will surround part of the robot before the competition, and then as soon as the robot starts its path, it will extend from the back of the robot via a rack and pinion mechanism. The trailer will drag behind the robot for the entirety for the competition until the robot reaches its final destination at the duck pond. Once the robot reaches the duck pond, it will drop the trailer off with all of the ducks inside and leave them behind by spinning the gear from the rack and pinion which will eject the trailer and ducks. This will save much needed space within the robot and save time when depositing the ducks at their duck pond location.

The size of the robot will lead to another constraint that will need to be taken into consideration which is the size of the silo for storing the pedestals. In order to save as much space as possible, the silo will only be tall enough to store three pedestals at once. Because of this constraint, there will need to be a “full” sensor that will tell the main controller to drop the pedestals as soon as possible. The plan is to drop them wherever the robot is located when three pedestals are collected. This way there is not a clog within the robot when more than three pedestals are collected. 

The motor selected for the trailer design should be able to push the weight of the ducks and the trailer itself when ejecting the trailer at the duck pond location. At worst case, the trailer will be carrying all ten ducks, so the weight will be 0.708 kg (the weight of all ten ducks) + XX (the weight of the trailer). The friction coefficient of rubber on steel is approximately 0.8.

The motor selected for the silo extension and pedestal drop off should be able to withstand the weigh of at most three pedestals and the weight of the silo itself. At worst case, the motor will have 0.0618 kg (the weight of three pedestals) + XX (the weight of the silo). 

## Electrical Schematic

The electrical schematic for the object storage subsystem is shown below.

## Analysis

## Buildable Schematic

![FunnelSideView](https://user-images.githubusercontent.com/112424739/202914982-efbea400-4a37-4475-895f-5980e5515300.png)

![FunnelTopView](https://user-images.githubusercontent.com/112424739/202914995-e1fec3a0-3223-471c-90d9-91331fba9701.png)


## BOM


