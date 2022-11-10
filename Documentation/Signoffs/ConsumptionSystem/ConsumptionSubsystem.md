# Consumption Subsystem Signoff

## Function of the Subsystem

The consumption subsystem’s main goal is to pick up any duck and pedestal that is in the predetermined path of the robot. Throughout the competition, the robot will make a complete path through the entire playing field that will be predetermined and designed by the team and come in contact with all of the ducks and pedestals that are present. The robot will need to pick these items up in order to later sort them and place them in their proper locations that are discussed within other subsystems. 

The consumption subsystem will intake the object via three rows of spinning spokes. The spokes will rotate against the body of the robot and take the captured ducks and pedestals upwards towards the top of the robot. The spokes will need to be a flexible material that can bend against the robot’s body. in order to save some space and ensure each item collected is taken upwards. The spokes will be held in place by PVC and will be spun by a DC brushed motor. A DC brushed motor was chosen because we will be supplying this subsystem with DC power, and these motors are less expensive than the alternative options. 


## Constraints

The second section should contain the constraints for the subsystem. The reasoning behind the constraints must be given. As a few examples, constraints may exist due to physics based limitations or requirements, other subsystem requirements, standards, ethics, or socioeconomic reasons. 

Each subsystem must have at least one constraint arising from standards, ethics, or socioeconomic well being.

## Buildable schematic 

The third section should show the buildable schematic directly embedded in the markdown file as a jpeg image. If the schematic is not clearly readable and appropriately sized, the supervisor will reject the signoff. 

The schematic must be appropriate to the design. ie. 3d model for a physical system or wiring schematic for a circuit. Further, the schematic(s) must contain every detail necessary for the design to be built by someone who has no knowledge of the design. Every relevant component value and measurement must be given.

The actual design artifacts (3d model, Cad files, gerber files, etc) must be uploaded to the documentation folder in the appropriate location.

## Analysis

A complete and relevant analysis of the design showing that it **should** meet the constraints and perform the desired function must be given. This analysis must be comprehensive and well explained so that it is convincing to the faculty supervisor. If the signoff request is not convincing either because the requirements and constraints are insufficient, unjustified, or not appropriately shown to be met by the design, then approval will not be given. Without approval, the components for the subsystem aren't allowed to be ordered. 

## BOM

A complete list of all components needed for the design must be given with the cost of each component and the total cost of the subsystem. The BOM should be a markdown table (excel tables can be copied and pasted directly into the markdown file and they will be automatically converted).

Name of Item	Description	Used in which subsystem(s)	Part Number	Manufacturer	Quantity	Price	Total
D Profile Shaft	D-Profile rotary shaft, D-profile ends, 1045 Carbon steel, 3/8'' Diameter, 12'' Long	Consumption	3832T1	McMaster-Carr	3	11.67	35.01
Bushing/Bearing	Ball bearing, Shielded, Trade Number R6-2Z for 3/8'' Shaft diameter	Consumption	60355K45	McMaster-Carr	6	6.25	37.5
Shaft Collar	Set Screw Shaft Collar for 3/8" Diameter, Black-Oxide 1215 Carbon Steel	Consumption	9414T8	McMaster-Carr	6	1.75	10.5
Shaft Pulley	Corrosion-Resistant Timing Belt Pulley, XL, 3/8" Maximum Width, Hub, 2 Flange, 1.75" OD, 3/8" Shaft	Consumption	1277N28	McMaster-Carr	3	16.65	49.95
Motor Pulley	Corrosion-Resistant Timing Belt Pulley, XL Series, 3/8" Maximum Belt Width, with Hub, 2 Flanges, 7/8" OD	Consumption	1277N41	McMaster-Carr	1	10.2	10.2
Timing Belt	XL Series Timing Belt, Trade No. 210xL025	Consumption	6484K219	McMaster-Carr	1	6.97	6.97
Motor 	47:1 Metal gearmotor 25Dx67L mm HP 6V with 48 CPR Encoder	Consumption	4805	Pololu	1	48.95	48.95
Washer Pack	316 Stainless Steel Washer for 3/8" Screw Size, 0.406" ID, 0.75" OD	Consumption	90107A127	McMaster-Carr	1	9.5	9.5
Corner Bracket	Corner Machine Bracket, Finish-Your-Own, 6061 Aluminum, 1" x 1" x 1-1/4"	Consumption	2313N36	McMaster-Carr	1	7.24	7.24
Side Profile	Fabricated by 3D printer	Consumption			2	0	0
Ramp Element	Fabricated by 3D printer	Consumption			1	0	0
TPU Spoke	Fabricated by 3D printer	Consumption			12	0	0
Forward Ejection	Fabricated by 3D printer	Consumption			1	0	0
Motor Mount	Fabricated by 3D printer	Consumption			1	0	0
Total				Total Components	40	Total Cost	215.82
![image](https://user-images.githubusercontent.com/30758520/201000765-1a53d3d8-f0fb-40a5-bc14-65480382e3b7.png)

