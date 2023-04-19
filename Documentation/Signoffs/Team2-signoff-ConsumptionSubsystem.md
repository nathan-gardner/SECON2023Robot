# Consumption Subsystem Signoff -> Delivery Subsystem Signoff (updated 4/19/2023)

## Function of the Subsystem

The consumption subsystem’s main goal is to pick up any duck and pedestal that is in the predetermined path of the robot. Throughout the competition, the robot will make a complete path through the entire playing field that will be predetermined and designed by the team and come in contact with all of the ducks and pedestals that are present. The robot will need to pick these items up in order to later sort them and place them in their proper locations that are discussed within other subsystems. 

### Function:
- Intake the object via three rows of spinning shafts with spokes
- Spokes (flexible material held in place with set screws on a D-profile carbon steel shaft) will push objects against the body of the robot and take the captured ducks and pedestals upwards towards the top of the robot with motion from a DC brushed motor
- Worst case scenerio of three ducks analyzed below

## Constraints

The consumption subsystem has a few constraints that it must abide by in order to be successful. The first constraint is that it must have a motor that has a minimum torque of $0.2168\ N \ast m$. The calculations used to find this minimum torque are laid out in the constraints portion of this document. The torque was determined based on the weight of the ducks on the spokes of the intake mechanism as well as the anticipated friction of the spokes pushing the ducks against the body of the robot and the floor. We found the weight of the ducks to be $70.8\ g$ and the friction constant of the rubber duck on the aluminum body of the robot to be $0.64$. The weight of the duck was considered because the weight of the pedestals is much less than that of the duck.

Taking the weight of the ducks into consideration, this places a constraint on the team’s decision of which material to use for the design. TPU will be used to build the spokes for the intake mechanism and use set screws on a D-profile carbon steel shaft to hold the spokes in place. The calculations used to determine these materials are shown below in the analysis portion of the document. The weight of the duck ( $70.8\ g$ ) had to be taken into consideration as well as the rigidity we need for both materials. The ducks weigh much more than the pedestals, so the duck’s weight was used for all calculations. The set screws that hold the spokes in place need to be rigid enough to securely hold all them while supporting multiple ducks in the worst case scenario.  

Assuming all three shafts are lifting a single duck, the maximum weight the motor will need to move would be $212.4\ g$. The acceleration of gravity ( $9.8\ m/s^2$ ), the friction constant of rubber against the aluminum side of the robot ( $0.64$ ), and the weight of the ducks ( $0.0708\ kg$ ) combined was used to find the force perpendicular to the moment arm of the spokes. Once the force needed based on the weight of the duck and the friction on the body of the robot was found, the force, the length of the spokes, and sine of the angle in which the items will be traveling were multiplied to find the torque needed to lift multiple ducks.

The ratio of the torque for each shaft is proportional to the ratio of the diameter of the two wheels either driving or being driven by the belt. The diameter of the driving wheel on the motor is 0.88 inches and the diameter of the three wheels being turned is 1.75 inches. That means the multiplication constant is 1.998. The torque for the three shafts being driven would therefore need to be 1.988 times what our calculations for the needed torque of the motor is, or $0.441 \ast 1.988$, which equals $0.877\ kg \ast cm$. This calculation is in the maximum torsional sheer strength calculation later in this report.

Additionally, the team needs to analyze whether or not the motor will meet the speed needed to pick up the items within the allotted amount of time. As previously mentioned, the motor will need to have a torque of $0.877\ kg \ast cm$. Assuming the motor provides 60 rpm as a worst-case scenario and the ducks are in full contact with the spokes for the entire rotation, the upwards velocity of the ducks would be approximately $11 \frac{ in }{ sec }$. Assuming the ducks need to travel 10 inches to make it to the conveyor belt, the ducks will make it to the top of the robot in 1.1 second. This value displays that even at a worst case, the spokes will be able to intake the objects at the speed necessary.

The next constraint that was considered was fitting the system within the size of the robot. The robot must fit in a 1’x1’x1’ area, so this adds the constraint of fitting each subsystem of this robot within the available space. The size of the consumption subsystem had to be such that the ducks would be accommodated. The duck’s size was considered more so than the pedestal because the duck has much larger measurements than the pedestal. The team’s object consumption design takes up $600\ inch^3$ including the motor. The analysis to find this value is shown below in the team’s 3D model of the system. 

Another constraint is that the robot must pick up all the objects in its path in a specific and constrained amount of time. This is essential to the robot's proper function because the round length is three minutes. Object consumption cannot take a large majority of the time because we need to allow for time to move the ducks to the pond and to activate the fireworks. Two minutes was determined as the maximum possible time that the robot will be allowed to consume objects around the arena and this means that the robot will need to move six inches per second. Two minutes was selected because it is the optimal balance which allows for time to transverse the arena at a reasonable speed ( $6\ \frac{inches}{sec}$ ) and allows for another minute to complete the duck delivery and firework activation. The path the robot will take is below and is total 480 inches long. This was analyzed to determine the amount of items the robot needs to consume in the two minutes. 

The object frequency analysis in the analysis section uses the velocity of the robot's movement and the amount of ducks and pedestals in the arena to perform a rough calculation of the object frequency. These calculations assume that the objects are completely evenly spread across the arena, which is the best approximation given the information we have from the competition explanation, that the objects will be randomly spread around the arena. This will likely not be exactly the case, but we are assuming complete randomness in analysis. 

![image](https://user-images.githubusercontent.com/30758520/202878257-abfdcb97-adcb-41a7-911d-a3a19630d73c.png)

The maximum shear strength calculations are below and were used in the selection of materials for the construction of the object consumption subsystem. The shear is the strain on a structure caused by pressure when its layers are laterally shifted in relation to each other. The maximum shear strength is of carbon steel is analyzed in the analysis portion below.  

Standard: OSHA Standard 1910.212(a)(1) states that guards are needed around moving parts. The robot’s consumption mechanism will be surrounded by walls on three of the four sides to account for this guard.

Conceptual Design Document: [here](https://github.com/nathan-gardner/CapstoneRepo/blob/main/Reports/Team2_ConceptualDesignandPlanningFinal.pdf)

## Buildable schematic 

Shown below is our buildable schematic design made in SOLIDWORKS. The spokes will rotate and intake any duck or pedestal in the robot’s path. The consumption mechanism will have three rows of spokes that will direct the objects upwards and inwards up a ramped wall to the beginning of the sorting mechanism. The back wall will have the ability to be adjusted within the testing phase in order to ensure that the spokes are close enough to the wall to properly intake each item. The spokes will be made of TPU and the rotating shafts will be made of pvc pipes for prototyping, but carbon steel for the final product. The ramp will be made of some kind of printed plastic for prototypes and aluminum for the final product. All calculations and analysis were done for the final product, not the prototype. All measurements shown on the 3D model are in inches. 

### Electrical Schematic

![image](https://user-images.githubusercontent.com/30758520/203184746-a586aec1-197b-4342-a30f-c6afef5b303f.png)

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

### Intake Mechanism inside box of robot (guards in place according to safety standard)

CAD Files for Object Consumption Mechanism: [here](https://github.com/nathan-gardner/CapstoneRepo/tree/main/Documentation/3D%20Models/ConsumptionSystem/Intake_Assembly_11-9-222)

## Analysis

### Motor Torque

![image](https://user-images.githubusercontent.com/30758520/201001312-4429694b-1b6a-4fd6-977f-bc88206feb7e.png)

The DC brushed motor chosen is part number #4805 from Pololu. The motor is considered high power and runs off of $6\ V$. The torque needed for the motor to supply in the worst case is $0.877\ kg \ast cm$. This torque value was found using a Simulink simulation shown below. The target rpm we want to run the motor at is $120\ rpm$, motor #4805 produces a torque of about $9.1\ kg \ast cm$, which is more than enough for the worst case scenario. The power needed for $120\ rpm$ is about $4\ W$ and will draw close to $2.75\ A$. The power subsystem will be designed to deliver adequate power to the motor used in this subsystem. The motor's gear ratio is 46.85:1 and uses a metal spur gear box internal to the motor. 

![image](https://user-images.githubusercontent.com/30758520/202034744-5f73f82d-9cde-43f5-84db-367644be47fe.png)

![image](https://user-images.githubusercontent.com/30758520/202034267-9fc5a4aa-4298-4bf2-a65b-7186fc19af5c.png)

The simulation above depicts the friction of the spokes against the body of the robot in the feedback loops on the left side of the model. The three forces from the ducks are added and the torque required to push those ducks is calulated and graphed on the oscilloscope as the output. The equation we used for the torque is $\tau = F \ast r \ast sin(\theta)$.

The safety factor for motor selection was two times the needed torque based on simulations. 

Motor torque Simulink Model: [here](https://github.com/nathan-gardner/CapstoneRepo/tree/main/Software/matlab-sim/ConsumptionSystem)

### Maximum Torsional Shear Strength 

### Torque ratio calculation

$\frac{ \tau_{shaft} }{ \tau_{drive} } = \frac{r_{shaft}}{r_{drive}}$

$\tau_{shaft} = \frac{r_{shaft} \ast \tau_{drive}}{r_{drive}}$

$\tau_{shaft} = 1.988 \ast \tau_{drive}$

$\tau_{shaft} = 0.877\ kg \ast cm$

Since the motor we selected is rated for up to $9.1 kg \ast cm$, it will meet the needs of the team since it exceeds the maximum rating of the motor. 

### Torsional Shear Strength Calculation

$\tau_{max}=\frac{16T}{\pi d^{3}}$

$T_{shaft} = 0.877\ kg \ast cm = 0.086\ N \ast m$
(torque at the top of the power peak considering gear ratio internal to the motor)

$d = 0.00635\ m$
(shaft diameter)

$= 1,710,600\ Pa$

$= 1.71\ MPa$

Normal carbon steel ranges between $260 - 500\ MPa$. Thus, the material chosen for the design will be sufficient. 

### Object Frequency Analysis

Stated in the contraints section, we are assuming the robot will transverse a 480 inch path in 120 seconds, and of that time, 80 seconds will be used for actual robot forward motion. Calculations for time per object consumed is below:

$f_{duck} = \frac{80\ sec}{10\ ducks} = 8 \frac{sec}{duck}$

$f_{pedestal} = \frac{80\ sec}{7\ pedestals} = 11.4 \frac{sec}{pedestals}$

$f_{object} = \frac{80\ sec}{17\ objects} = 4.7 \frac{sec}{object}$

### Motor Speed Intake Analysis

Assuming a worst case scenario of 60 rpm and each item staying in full contact with the spokes for the entire ride up the intake, the following calculations were made.

$\omega = 60 rpm = 2 \ast \pi \frac{ rad }{ sec }$

$v = r \ast \omega = 1.75 \ast 2 \ast \pi = 10.996 \frac{ in }{ sec }$

Since we need to pick up items at a rate of $4.7 \frac{sec}{object}$, this expected and absolute worst case scenario will still be enough to push the items up at a rate in which the robot will be able to collect all the items. 

## BOM

The bill of materials for the consumption mechanism is below. This includes all parts that are needed for design and manufacture of the robot. 

| Name of Item     | Description                                                                                              | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
|------------------|----------------------------------------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|--------|
| D Profile Shaft  | D-Profile rotary shaft, D-profile ends, 1045 Carbon steel, 3/8'' Diameter, 12'' Long                     | Consumption                | 3832T1      | McMaster-Carr    | 3        | 11.67      | 35.01  |
| Bushing/Bearing  | Ball bearing, Shielded, Trade Number R6-2Z for 3/8'' Shaft diameter                                      | Consumption                | 60355K45    | McMaster-Carr    | 6        | 6.25       | 37.5   |
| Shaft Collar     | Set Screw Shaft Collar for 3/8" Diameter, Black-Oxide 1215 Carbon Steel                                  | Consumption                | 9414T8      | McMaster-Carr    | 6        | 1.75       | 10.5   |
| Shaft Pulley     | Corrosion-Resistant Timing Belt Pulley, XL, 3/8" Maximum Width, Hub, 2 Flange, 1.75" OD, 3/8" Shaft      | Consumption                | 1277N28     | McMaster-Carr    | 3        | 16.65      | 49.95  |
| Motor Pulley     | Corrosion-Resistant Timing Belt Pulley, XL Series, 3/8" Maximum Belt Width, with Hub, 2 Flanges, 7/8" OD | Consumption                | 1277N41     | McMaster-Carr    | 1        | 10.2       | 10.2   |
| Timing Belt      | XL Series Timing Belt, Trade No. 210xL025                                                                | Consumption                | 6484K219    | McMaster-Carr    | 1        | 6.97       | 6.97   |
| Motor            | 47:1 Metal gearmotor 25Dx67L mm HP 6V with 48 CPR Encoder                                                | Consumption                | 4805        | Pololu           | 1        | 48.95      | 48.95  |
| Motor Controller | TB9051FTG Single Brushed DC Motor Driver Carrier                                                         | Consumption                | 2997        | Pololu           | 1        | 11.95      | 11.95  |
| Washer Pack      | 316 Stainless Steel Washer for 3/8" Screw Size, 0.406" ID, 0.75" OD                                      | Consumption                | 90107A127   | McMaster-Carr    | 1        | 9.5        | 9.5    |
| Corner Bracket   | Corner Machine Bracket, Finish-Your-Own, 6061 Aluminum, 1" x 1" x 1-1/4"                                 | Consumption                | 2313N36     | McMaster-Carr    | 1        | 7.24       | 7.24   |
| Side Profile     | Fabricated by 3D printer                                                                                 | Consumption                |             |                  | 2        | 0          | 0      |
| Ramp Element     | Fabricated by 3D printer                                                                                 | Consumption                |             |                  | 1        | 0          | 0      |
| TPU Spoke        | Fabricated by 3D printer                                                                                 | Consumption                |             |                  | 12       | 0          | 0      |
| Forward Ejection | Fabricated by 3D printer                                                                                 | Consumption                |             |                  | 1        | 0          | 0      |
| Motor Mount      | Fabricated by 3D printer                                                                                 | Consumption                |             |                  | 1        | 0          | 0      |
| Total            |                                                                                                          |                            |             | Total Components | 41       | Total Cost | 227.77 |
