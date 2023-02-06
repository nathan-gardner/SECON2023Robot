# Feeding Subsystem

## Function

- Deliver the correct chip to the correct area based on color
  - Color will be detected with the color sensor from vision subsystem

There will be rectangles in the left corners of the arena which will be green (0x00FF00) and red (0xFF0000). Green signifies the manatees and red signifies the alligators aquarium. A color sensor will be pointed towards the ground and will detect the color of the aquarium, and spin the servo the correct direction in order to drop the right color chip. This will allow the servo positions to be preprogrammed, so that the robot can react correctly to the colored aquariums. The servo will be connected to a plate that will hold the chips up, but when the servo is actuated in the correct direction, the chips will fall through a hole onto the floor of the arena and into the correct aquarium. The chips will sit in a sort of silo, and will be held up by a plate connected to the servo. The servo will rotate clockwise or counterclockwise, which will allow the chips to fall through a false bottom and onto the aquarium area. 

The position of the aquariums can be reflected based on the arena we are playing in, this is the reason for the design choice of making the robot detect the color before actuating and allowing the chips through a hole which will drop the chips in the aquarium. 

## Constraints

- The size of this subsystem will have to be such that it takes up as minimal space as possible, that is to fit the chips and allow for as much space for the rest of the components on the robot. Analysis for the allowable size of feeding system is in analysis. 
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

The dispensers that the chips will be stored in must be able to fit the chips. There will be three chips of each color, so the chips must be able to be stacked to a height of three in each silo. 

Chip Size $= 1.5 in \ast 0.1 in$
Dispenser Interior Size $= 4.5 in \ast 0.15 in \ast 1.5 in$

Therefore, the chips will comfortably fit inside the tubes. This part will also be 3D printed, so the sizing can be adjusted during testing. The chips will fit into the silos in a way similar to how chips fit into a game of Connect 4, they will be top loaded and sit so that the diameter of the chip, $1.5 inches$, is the height of the chip when it is loaded into the silo. 

### Motor Torque

The motor will only need to push the weight of the small 3D printed plate. The volume of the 3D printed chips are calculated and modeled as a cylinder below:

$V = \pi r^2 h = \pi \ast 0.75^2 \ast 0.1 = 0.176715\ in^3$

The density of the TPU is $0.0448\ \frac{lb}{in^3}$, so the weight of the TPU printed chip will be $0.0448\ \frac{lb}{in^3} \ast 0.176715\ in^3 = 0.00792 lb = 3.59\ grams$. 

The downward force of the chips on the plate will be the mass of three chips times the acceleration of gravity squared. This would be $0.01077\ kilograms \ast 9.81^2 \frac{m}{s^2}^2 = 1.0365\ N$. The torque needed for the servo will therefore be $\tau = 1.0365\ N \ast 0.047625\ m = 0.0494 N*m = 6.996 oz*in$, and this based on the worst case moment arm measurement, derived from the picture below.

INSERT IMAGE OF MOMENT ARM

The torque of the motor selected is 21 oz-in, so it will be more than sufficient for its cause because it is three times needed torque from the calculations above.

## BOM
| Name of Item           | Description                                        | Used in which subsystem(s)                                        | Part Number | Manufacturer | Quantity | Price | Total |
|------------------------|----------------------------------------------------|-------------------------------------------------------------------|-------------|--------------|----------|-------|-------|
| Servo Motor            | FEETECH FS90 Micro Servo                    | Feeding                                                           |2818     | Pololu       | 1        | 5.25 | 5.25 |
| Servo Motor Controller | Micro Maestro 6-Channel USB Servo Controller       | Feeding, Pedestal Storage, and Duck Storage (included on Pedestal BOM) | 1350        | Pololu       | 1        | 0     | 0     |
| Chip Tube              | 3D printed 1.65 in x 0.5 in tube to hold the chips | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |
| Chip Stopper           | 3D printed plate to stop the chips from falling    | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |


