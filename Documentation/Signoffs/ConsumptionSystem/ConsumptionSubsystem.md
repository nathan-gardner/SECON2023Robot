# Consumption Subsystem Signoff

## Function of the Subsystem

The consumption subsystem’s main goal is to pick up any duck and pedestal that is in the predetermined path of the robot. Throughout the competition, the robot will make a complete path through the entire playing field that will be predetermined and designed by the team and come in contact with all of the ducks and pedestals that are present. The robot will need to pick these items up in order to later sort them and place them in their proper locations that are discussed within other subsystems. 

The consumption subsystem will intake the object via three rows of spinning shafts with spokes. The spokes will push objects against the body of the robot and take the captured ducks and pedestals upwards towards the top of the robot. The spokes will be a flexible material that can slightly contour to the shape of the duck’s body in order to ensure each item collected is taken upwards. The spokes will be held in place by set screws on a D-profile carbon steel shaft and will be spun by a DC brushed motor. A DC brushed motor was chosen because the motor will be supplied with DC power, and these motors are less expensive than the alternative options. The exact motor that will be used and the explanation will be stated in the analysis section.

## Constraints

The consumption subsystem has a few constraints that it must abide by in order to be successful. The first constraint is that it must have a motor that has a minimum torque of 0.2168 N-m. The calculations used to find this minimum torque are laid out in the analysis portion of this document. The torque was determined based on the weight of the ducks on the spokes of the intake mechanism as well as the anticipated friction of the spokes pushing the ducks against the body of the robot and the floor. We found the weight of the ducks to be 70.8 g and the friction constant of the rubber duck on the aluminum body of the robot to be 0.64. The weight of the duck was considered because the weight of the pedestals is much less than that of the duck.

Taking the weight of the ducks into consideration, this places a constraint on the team’s decision of which material to use for the design. TPU will be used to build the spokes for the intake mechanism and use set screws on a D-profile carbon steel shaft to hold the spokes in place. The calculations used to determine these materials are shown below in the analysis portion of the document. The weight of the duck (70.8 g) had to be taken into consideration as well as the rigidity we need for both materials. The ducks weigh much more than the pedestals, so the duck’s weight was used for all calculations. The set screws that hold the spokes in place need to be rigid enough to securely hold all them while supporting multiple ducks in the worst case scenario.  

The next constraint that was considered was fitting the system within the size of the robot. The robot must fit in a 1’x1’x1’ area, so this adds the constraint of fitting each subsystem of this robot within the available space. The size of the consumption subsystem had to be such that the ducks would be accommodated. The duck’s size was considered more so than the pedestal because the duck has much larger measurements than the pedestal. The team’s object consumption design takes up 600 inch3 including the motor. The analysis to find this value is shown below in the team’s 3D model of the system. 

Standard: OSHA Standard 1910.212(a)(1) states that guards are needed around moving parts. The robot’s consumption mechanism will be surrounded by walls on three of the four sides to account for this guard.

Ethics: The consumption mechanism will be designed such that no one that comes into contact with the rotating spokes of the robot will be harmed. We will do this by making the spokes out of a soft, flexible material that will not cause physical damage to any object or person in its path.

Socioeconomic: This subsystem will help Tennessee Tech make a good impression on the stage of the SoutheastCon hardware competition. This will improve the public image of Tennessee Tech and will potentially attract new students.

## Buildable schematic 

Shown below is our buildable schematic design made in SOLIDWORKS. The spokes will rotate and intake any duck or pedestal in the robot’s path. The consumption mechanism will have three rows of spokes that will direct the objects upwards and inwards up a ramped wall to the beginning of the sorting mechanism. The back wall will have the ability to be adjusted within the testing phase in order to ensure that the spokes are close enough to the wall to properly intake each item. The spokes will be made of TPU and the rotating shafts will be made of pvc pipes for prototyping, but carbon steel for the final product. The ramp will be made of some kind of printed plastic for prototypes and aluminum for the final product. All calculations and analysis were done for the final product, not the prototype. All measurements shown on the 3D model are in inches. 

### Intake Mechanism Back
![IntakeMechanismBack](https://user-images.githubusercontent.com/30758520/201001784-44b5ea8e-67ed-4592-b330-f094e41a86d7.png)

### Intake Mechanism Front
![IntakeMechanismFront](https://user-images.githubusercontent.com/30758520/201001806-3ad95a88-a283-444a-a36c-ce11eff1b79e.png)

### Intake Mechanism Front Angle
![IntakeMechanismFrontAngle](https://user-images.githubusercontent.com/30758520/201001842-7290913f-ed47-4942-937a-cfcf2d664771.png)

### Intake Mechanism Side
![IntakeMechanismSide](https://user-images.githubusercontent.com/30758520/201001851-18215d47-ae5f-426b-ba2e-bb4d5252f6ea.png)

### Intake Mechanism Side
![IntakeMechanismSide2](https://user-images.githubusercontent.com/30758520/201001860-1a9735b8-17e1-4fd6-9f2a-2def6dbd8cc9.png)

## Analysis

### Motor Torque

![image](https://user-images.githubusercontent.com/30758520/201001312-4429694b-1b6a-4fd6-977f-bc88206feb7e.png)

The DC brushed motor chosen is part number #4805 from Pololu. The motor is considered high power and runs off of 6 V. The torque needed for the motor to supply in the worst case is 0.2168 N-m (or 22.11 kg*mm). This torque value was found using a Simulink simulation shown below. The target rpm we want to run the motor at is 120 rpm, motor #4805 produces a torque of about 40 kg*mm, which is more than enough for the worst case scenario. The power needed for 120 rpm is about 4 W and will draw close to 2.75 A. The power subsystem will be designed to deliver adequate power to the motor used in this subsystem.

![image](https://user-images.githubusercontent.com/30758520/201001401-8d3bfba7-8820-4853-a2f6-23a22c52e0e4.png)

Assuming all three shafts are lifting a single duck, the maximum possible weight the motor will need to move would be 212.4 g. The acceleration of gravity (9.8 m/s2), the friction constant of rubber against the aluminum side of the robot (0.64), and the weight of the ducks (0.0708 kg) combined was used to find the force. Once the force needed based on the weight of the duck and the friction on the body of the robot was found, the force, the length of the spokes, and sine of the angle in which the items will be traveling were multiplied to find the torque needed to lift multiple ducks. The safety factor for motor selection was two times the needed torque based on simulations. 

### Maximum Torsional Shear Strength 

![image](https://user-images.githubusercontent.com/30758520/201001499-2f0068d9-8bb0-4432-b3ba-53a41303c4c5.png)

T = 0.441 N-m (torque at the top of the power peak)

d = 0.00635  m (shaft diameter)

= 8,771,775.8 Pa

= 8.77 MPa

Normal carbon steel ranges between 260 - 500 MPa. Thus, the material chosen for the design will be sufficient. 

## BOM

The bill of materials for the consumption mechanism is below. This includes all parts that are needed for design and manufacture of the robot. 

![image](https://user-images.githubusercontent.com/30758520/201002485-b2cc754a-2fcc-4ee5-9257-ba2f22b33b48.png)

