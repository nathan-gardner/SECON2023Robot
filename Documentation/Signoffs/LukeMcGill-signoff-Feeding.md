# Feeding Subsystem - revised 4/6/2023

## Function 

~~- Deliver the correct chip to the correct area based on color~~

~~- Color will be detected with the color sensor from vision subsystem~~

~~There will be rectangles in the left corners of the arena which will be green (0x00FF00) and red (0xFF0000). Green signifies the manatees and red signifies the alligators aquarium. A color sensor will be pointed towards the ground and will detect the color of the aquarium, and spin the servo the correct direction in order to drop the right color chip through the false bottom on the feeding subsystem. This will allow the servo positions to be preprogrammed, so that the robot can react correctly to the different colored aquariums. The servo will be connected to a plate that will hold the chips up, but when the servo is actuated to a specific orientation, the chips will fall through a hole onto the floor of the arena and into the correct aquarium. The chips will sit in a sort of silo, and will be held up by a plate connected to the servo. The servo will rotate clockwise or counterclockwise, which will allow the chips to fall through a false bottom and onto the aquarium area.~~

~~The position of the aquariums can be reflected based on the arena we are playing in, this is the reason for the design choice of making the robot detect the color before dispersing the chips onto the playing field.~~

- Deliver the correct chip to the correct area

There will be rectangles in the left corners of the arena which will be green and red. Green signifies the manatees and red signifies the alligators aquarium. The robot will drop the chips in their respective areas based on the teams' placement before the competition. The servos will activate when they reach their pre-determined destination on each animal area. There will be two servos mounted on either side of the robot that will hold either red or green chips in a small "cup". Once they reach their destinations, the servos will turn about 180&deg; and dump the chips out of the cups, leaving the chips behind.

The competition rules allow for the feeding chips to be preloaded into the robot, so someone on the team will load the color chips into the feeding subsystem before the start of each round. One of the two silos will be loaded with green chips and the other of the two silos will be loaded with red chips. 


## Constraints

~~- The size of this subsystem will have to be such that it takes up only enough space to fit the chips (6.15 $in^3$) and allow space for the rest of the components on the robot. Analysis for the allowable size of feeding system is in analysis, with CAD models showing subsystem fit within the robot.~~

- The servos with the chip cups attached will be mounted towards the top of the robot in order to allow for some much needed space within the robot. The cups they are held in are 1.5 in diameter by 5 in height. Since they are mounted towards the outer edge on the top, they will not be taking up much space inside the robot.
- The servo motors must be as small as possible, while also providing necessary torque. The torque needed is 0.0494 $N \ast m$, so micro servos can be used. The calculation for the needed torque is below in analysis.

## Buildable Schematic

![image](https://user-images.githubusercontent.com/112424739/215866478-37b8d6f1-c0f0-4542-9adf-73eb05a63cd0.png)

A closer look at the plate that the chips will sit on is below.

![image](https://user-images.githubusercontent.com/112424739/215889365-92003613-8937-4743-aeb3-278711afe4fa.png)

The layout of the chassis is below showing the major subsystems and their placement within the robot. This subsystem is titled "chip dispenser" or "chips". 

![chassis](https://user-images.githubusercontent.com/112424739/216873478-79e741be-d40a-422f-9905-b26dc7f066d7.png)

Below are images showing how all of the subsystems, including the feeding subsystem will fit into the robot.

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

Chassis Subsystem Fit CAD model files available [here](https://github.com/nathan-gardner/CapstoneRepo/tree/main/Documentation/3D%20Models/Chassis/Chassis_Fit).

## Electrical Schematic

The electrical schematic for the feeding subsystem is shown below. 

![image](https://user-images.githubusercontent.com/112424739/218158290-946fe6fc-cd54-4bf9-94a0-50b784b2db8b.png)

## Analysis

### Size

~~The dispensers that the chips will be stored in must be able to fit the chips. There will be three chips of each color, so the chips must be able to be stacked to a height of three in each silo.~~ 

Chip Size $= 1.5 in \ast 0.1 in$

~~Dispenser Interior Size $= 4.5 in \ast 0.15 in \ast 1.5 in$~~

Dispenser Interior Size $= \pi \ast 1.5 in \ast 0.5 in = 0.88 in^3$

Therefore, the chips will comfortably fit inside the cups. ~~This part will also be 3D printed, so the sizing can be adjusted during testing. The chips will fit into the silos in a way similar to how chips fit into a game of Connect 4, they will be top loaded and sit so that the diameter of the chip, $1.5 inches$, is the height of the chip when it is loaded into the silo.~~ 

### Motor Torque

The motor will only need to push the weight of the ~~small 3D printed plate.~~ cups with the chips inside. The volume of the 3D printed chips are calculated and modeled as a cylinder below:

$V = \pi r^2 h = \pi \ast 0.75^2 \ast 0.1 = 0.176715\ in^3$

The density of the TPU is $0.0448\ \frac{lb}{in^3}$, so the weight of the TPU printed chip will be $0.0448\ \frac{lb}{in^3} \ast 0.176715\ in^3 = 0.00792 lb = 3.59\ grams$. 

The downward force of the chips on the plate will be the mass of three chips times the acceleration of gravity squared. This would be $0.01077\ kilograms \ast 9.81^2 \frac{m}{s^2}^2 = 1.0365\ N$. The torque needed for the servo will therefore be $\tau = 1.0365\ N \ast 0.047625\ m = 0.0494\ N \ast m = 6.996\ oz \ast in$, and this based on the worst case moment arm measurement, derived from the picture below.

![image](https://user-images.githubusercontent.com/30758520/216861296-9122d564-b9b9-44e5-833d-dd81255a06ec.png)

The torque of the motor selected is 21 oz-in, so it will be more than sufficient for its cause because it is three times minimum necessary torque from the calculations above. 

## BOM
| Name of Item           | Description                                        | Used in which subsystem(s)                                        | Part Number | Manufacturer | Quantity | Price | Total |
|------------------------|----------------------------------------------------|-------------------------------------------------------------------|-------------|--------------|----------|-------|-------|
| Servo Motor            | FEETECH FS90 Micro Servo                    | Feeding                                                           |2818     | Pololu       | 1        | 5.25 | 5.25 |
| Servo Motor Controller | Micro Maestro 6-Channel USB Servo Controller       | Feeding, Pedestal Storage, and Duck Storage (included on Pedestal BOM) | 1350        | Pololu       | 1        | 0     | 0     |
| Chip Tube              | 3D printed 1.65 in x 0.5 in tube to hold the chips | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |
| Chip Stopper           | 3D printed plate to stop the chips from falling    | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |


