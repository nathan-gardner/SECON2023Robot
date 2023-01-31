# Feeding Subsystem

## Function

- Deliver the correct chip to the correct area based on color
  - Color will be detected with the color sensor from vision subsystem

There will be rectangles in the left corners of the arena which will be green (0x00FF00) and red (0xFF0000). Green signifies the manatees and red signifies the alligators aquarium. A color sensor will be pointed towards the ground and will detect the color of the aquarium, and spin the servo the correct direction in order to drop the right color chip. This will allow the servo positions to be preprogrammed, so that the robot can react correctly to the colored aquariums. The servo will be connected to a plate that will hold the chips up, but when the servo is actuated in the correct direction, the chips will fall through a hole onto the floor of the arena and into the correct aquarium. The chips will sit in a sort of silo, and will be held up by a plate connected to the servo. The servo will rotate clockwise or counterclockwise, which will allow the chips to fall through a false bottom and onto the aquarium area. 

The position of the aquariums can be reflected based on the arena we are playing in, this is the reason for the design choice of making the robot detect the color before actuating and allowing the chips through a hole which will drop the chips in the aquarium. 

## Constraints

- The size of this subsystem will have to be such that it takes up as minimal space as possible
- The servo motor must be as small as possible since it only has to push the weight of a small 3D printed plate
- The color sensor must be able to detect the red and green animal enclosures
- The chips must be able to fit within their dispensers
- The chip delivery will be on the opposite side as the pedestal storage silos to allow for a simple delivery to each enclosure

## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/215866478-37b8d6f1-c0f0-4542-9adf-73eb05a63cd0.png)

A closer look at the plate that the chips will sit on is below.

![image](https://user-images.githubusercontent.com/112424739/215889365-92003613-8937-4743-aeb3-278711afe4fa.png)

## Electrical Schematic
The electrical schematic for the feeding subsystem is shown below. 

![image](https://user-images.githubusercontent.com/112424739/215889260-5daf72f9-51eb-45ce-a533-0d5d7dbb871a.png)

## Analysis

### Size

The dispensers that the chips will be stored in must be able to fit the chips.

Chip Size $= 1.5 in \ast 0.1 in$
Dispenser Interior Size $= 4.5 in \ast 0.15 in \ast 1.5 in$

Therefore, the chips will comfortably fit inside the tubes. This part will also be 3D printed, so the sizing can be adjusted during testing.

### Motor Torque

The motor will only need to push the weight of the small 3D printed plate.

The torque of the motor selected is 21 oz-in, so it will be more than sufficient for its cause.

## BOM
| Name of Item           | Description                                        | Used in which subsystem(s)                                        | Part Number | Manufacturer | Quantity | Price | Total |
|------------------------|----------------------------------------------------|-------------------------------------------------------------------|-------------|--------------|----------|-------|-------|
| Servo Motor            | FEETECH FS90 Micro Servo                    | Feeding                                                           |2818     | Pololu       | 1        | 5.25 | 5.25 |
| Servo Motor Controller | Micro Maestro 6-Channel USB Servo Controller       | Feeding, Pedestal Storage and Delivery (included on Pedestal BOM) | 1350        | Pololu       | 1        | 0     | 0     |
| Chip Tube              | 3D printed 1.65 in x 0.5 in tube to hold the chips | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |
| Chip Stopper           | 3D printed plate to stop the chips from falling    | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |


