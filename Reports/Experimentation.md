This document should be a pdf that details all experiments which were conducted. The purpose of the experiment, a description of the experiment, the expected result of the experiment, and the number of trials (must be enough to establish statistically significant results) must be given for each of the experiments.

The document should be in IEEE format.

The results of the experiments should be in the proper format. Data that is best suited to graphs must be in a graph. The same is true of tables. Results should not be given in paragraph format. 


# **Experimental Analysis**

### **Cut Subsystems and Justification**

The following subsystems have been cut since the beginning of the project:
- Sorting
    - Due to size constraints and the parts for this subsystem not arriving, this subsystem was cut.
- Duck Storage and Delivery
    - The project was rescoped before the competition and the duck storage and delivery subsystem did not make it into the final implementation. This decision was made unanimously by the team at the competition. It was determined that the duck storage and delivery system, while well designed and theoretically functional, could not be reliably implemented before the competition. 
- Pedestal Storage and Delivery
    - Due to size constraints and the parts for this subsystem not arriving, this subsystem was cut.
- Vison
    - Due to the parts for this subsystem not arriving, this subsystem was cut.
- Consumption (repurposed - now called Delivery Subsystem)
    - The project was rescoped before the competition and the consumption subsystem did not make it into the final implementation. This decision was made unanimously by the team at the competition. The team decided to transition the consumption subsystem into the delivery subsystem. The direction of the motor was changed to push instead of consume, and the subsystem was used to push what items had been collected into the recycling area.


The following shall statements were closely related to these subsystems and therefore will not be discussed in this experimental analysis document:

- Shall design a robot which can find and move 90% of
the ducks into a holding area connected to the robot.
- Shall locate the duck pond in the center of the arena
within plus or minus one inch of error tolerance.
- Shall transport and place 90% of the ducks stored inside
the holding area to their final location in the duck pond.
- Shall find and move at least five pedestals into an internal
holding area inside the robot.
- Shall assemble one statue that is three pedestals tall and
one statue that is two pedestals tall using all five pedestals
obtained in order to maximize points obtained based on
discussion in weekly meetings.
- Shall place statues entirely inside the white inner circles
within plus or minus one inch of error tolerance.
- Shall place remaining unused pedestals that are held after
the five required pedestals have been obtained within the
internal holding area inside the robot.
- Shall move the extra pedestals obtained over the five
required pedestals in the recycling area.

## **Constraints from Signoffs:**

### **Feeding:**
- The design has been changed to two separate cups with two separate servo motors in order to simplify the design. The cups are only big enough to hold the chips, and are mounted on the servo motors which are mounted on the outer edge of the robot, thus saving much needed space inside the robot.

The food chip dispensers can be seen in the image below. They are located on the top crossbar of the robots chassis. They consist of a cup connected to a servo motor.

![image](/Documentation/Images/finalcadmodel.png)

Below is the final experimentation taken during the competition, we consistently delivered food chips as outlined in our project proposal. 

| Round | Green chip | Red chip |
| ----- | ---------- | -------- |
| 1     | 3          | 3        |
| 2     | 3          | 3        |
| 3     | 2          | 3        |
| 4     | 2          | 2        |
| 5     | 3          | 3        |
| 6     | 2          | 2        |

- Since the design has been simplified in order to save time, the servos do not need to have as much torque. However, they have much more than sufficient torque needed to flip the cups and dump the chips.

### **Locomotion:**
- **Weigh the robot**

The final competition robot weight was $14\ pounds\ or\ 6.35029\ kg\ \lt\ \approx\ 10.376\ kg$. 

- **Need to see how fast the robot can go**

***Max Speed***

$\frac{150\ rotations}{1\ min} * \frac{1\ min}{60\ sec} * \frac{48\pi\ mm}{1\ rotation} * \frac{0.00328084\ feet}{mm} = 1.24 \frac{feet}{sec} \gt 0.0677 \frac{meter}{sec} = 0.222 \frac{feet}{second}\ (minimum\ speed\ requirement)$

***Typical Competition Speed***

$\frac{100\ rotations}{1\ min} * \frac{1\ min}{60\ sec} * \frac{48\pi\ mm}{1\ rotation} * \frac{0.00328084\ feet}{mm} = 0.825 \frac{feet}{sec}$


### **Power:**

- The battery is sufficient in providing 12 V and over 2 A to meet the robot’s needs. This was achieved with the original 12 V TalentCell battery as well as 2 6 V batteries.

- The output of the battery is regulated by the DC-DC Converter and provides a constant 11.94 V to sufficiently power all components connected to it including the buck converter to the servo motors and the buck converter connected to the Jetson.
![image](https://user-images.githubusercontent.com/112428796/233485301-ef483440-b9e7-4346-90b2-16bede1f2558.png)


- Fuses were not implemented in the final design.

- The 6 V servos were powered via a buck converter. As you can see below, there is a photo of both the buck converters as well as the filtering circuits in order to filter the ripple voltage on the output. 
![image](https://user-images.githubusercontent.com/112428796/233485511-202565f5-ed1d-4c17-b028-e54fba6b84a1.png)

![image](https://user-images.githubusercontent.com/112428796/233485550-aee62d63-ce5a-4c24-92ce-f285168b67c4.png)


- The inductive load from the motors did not seem to be an issue, so the team decided to not add the capacitors on the input of the motors.

- The main power bus in the robot was supplied from the two 6 V batteries in series. This bus was connected to all the DC motor drivers. The first 6 V battery was used to power the 6 V motor driver for the delivery subsystem.  Seperate connections were made for powering the Jetson and the servo motors, which were powered from the 12 V battery.

- The team was having issues with the Jetson giving an undercurrent message from the 5 V USB output despite the analysis showing that it would be enough. The decision was made to implement another buck converter in order to deliver 5.20 V to supply the Jetson from the regulated 12 V output.

The undercurrent message was actually a general error message, and the issue was an undervoltage condition. This was cause by the large amount of current the Jetson draws and the resistance of the charging wire inducing a voltage drop across the wire. This is the reason that the team set the buck converter to generate 5.20 V to supply the Jetson. 

![image](https://user-images.githubusercontent.com/112428796/233485365-55536ce3-dec1-4e8b-aae2-10ea04deed95.png)

### **Low-Level Controller:**

- The Arduinos had more than enough GPIO especially with all of the sensors not being shipped in time. The team was able to cut down to using only one Arduino instead of the expected two Arduinos from the original detailed design. This saved space in the final implementation and simplified the final implemented design. 

### **Top-Level Controller:**

- The top-level controller is being fed about 5.2 V ***(have picture)*** from the battery and through a buck converter via the barrel jack.

- The serial communication between the Arduino and Jetson ran a 115200 baud, which is the max speed that can be reliably accomplished with the Arduino Mega2560. 

### **Delivery Subsystem (was called Consumption):**

- The team did not use the initially purchased 6 V motor (because it was too slow), and used a faster motor that was available in the capstone lab. 

- Many variations of spokes have been tested in order to see which ones work the best. The only spokes that made it into the final implementation were curved TPU spokes, and those can be seen in the final total robot CAD model in this document. 

- The size of the final consumption implementation was 5.25"x9.25"x11.75" (LxWxH). The consumption mechanism is large enough for a duck to be consumed as well as the pedestals.

In the final implementation in the competition, no ducks or pedestals were consumed. The motor direction of the consumption motor was reversed and the consumption was converted into a pusher. The robot collected objects in the arena on the consumption ramp, and then the spokes pushed out the objects into the recycling. 

- As mentioned previously, the path of the robot has changed due to the vision subsystem not being able to be implemented. The path created was based on encoder clicks and was always relative to the starting area. The robot did a point by point path driving in directions for a curtain number of encoder clicks. 

- There are three walls surrounding the intake in order to protect any limbs from moving parts. This was a safety feature implementation based on considerations made during detailed design. 

### **Fireworks:**

- The robot has the force necessary to flip the switch without issue, though it did not flip the switch during the competition (***ADD VIDEO***)

## **Measures of Success from Project Proposal**
- The team rescoped and decided not to start using the LED indicator to instead focus on other subsystems. A start switch was added to replace this functionality. (***ADD VIDEO***)

- The robot was unable to drop all the chips in the correct location on every competition run. (***ADD VIDEO***) (***Insert table showing how many chips were counted***)

- The project rescoped to remove duck storage and therefore did not collect and store any ducks. They will instead be pushed to the recycle bin

- The robot was unable to flip the light switch during the six competition rounds. The python script has been written as well as the video has been made as of March 10, 2023.

- Due to the sorting, consumption, and pedestal storage subsystems being cut or repurposed, the pedestals will no longer be consumed, sorted, or stacked. Instead, pedestals will be pushed to recycling just like the ducks.

- The robot's path was altered to push as many objects as possible into the recycle bin, instead of placing any remaining items into the recycle bin

## **Shall Statements**

### **Power:** 

- Shall design an autonomous robot with a single start
button, allowing the robot to start moving through its
environment.
    - The start button was implemented as a start switch. This starts the path for the robot 
- The robot will have a single emergency stop button at a
point that is easily accessible and can be safely reached,
which will shut down all physical movement performed
by the robot in the case of an emergency.
    - The E-stop button is easily accessible on the top of the robot. The switch cuts all power to the dc motors for locomotion and delivery.
- Shall create an easily reachable (not blocked by motors,
chassis, wheels, or any other object) emergency cut off
switch to allow the team to disable the robot in the case
of an emergency.
    - The E-stop button is easily accessible on the top of the robot.
- Shall have a self-latching emergency stop push-button
that has a positive operation. The button shall not be a
graphical representation or a flat switch based on NFPA
79 - 10.7.2. [1] This constraint addresses the need for the
addition of practical engineering standards.
    - The E-stop button chosen meets these specifications.
    
![image](https://user-images.githubusercontent.com/112428796/233485670-f1d4f3da-88d5-4cbe-b57c-d3cf5355f9d6.png)

- Shall abide by the Department of Energy Standard 79
FR 7845 in the team’s purchase or design of wall warts
for energy conservation and efficiency. [3] This constraint
addresses an ethical consideration by better ensuring the
safety of the team and all others interacting with the robot
as well as the addition of ethical standards.

### **Software:**

- Shall represent knowledge using IEEE standard IEEE
1872-2015 Ontologies for Robotics and Automation used
to represent knowledge about the typography of the arena.
This ontology will be used to represent relationships
between the landmarks in the area and what is known.
It should not change during the competition, so it can be
predefined. [2]

We ended up using encoder distances to represent the movement and position in the arena. This was written during the conceptual design when we had been considering using SLAM to navigate the arena area. 

### **Feeding:**

- Shall design an autonomous robot that will earn all
possible points for delivering 100% of the correct food
chips to both the manatees and alligators.

This was accomplished in the first two rounds of the competition, but was not accomplished in the later rounds of the competition. 

| Round | Green chip | Red chip | Cylinder Recycle | Duck Recycle | Duck Pond |
| ----- | ---------- | -------- | ---------------- | ------------ | --------- |
| 1     | 3          | 3        | 1                | 4            | 0         |
| 2     | 3          | 3        | 0                | 0            | 0         |
| 3     | 2          | 3        | 0                | 0            | 0         |
| 4     | 2          | 2        | 4                | 6            | 0         |
| 5     | 3          | 3        | 4                | 1            | 0         |
| 6     | 2          | 2        | 1                | 2            | 1         |

### **Fireworks:** 
- Shall design an autonomous robot that will be able to flip
a switch from left to right.

(**ADD FIREWORKS FLIP SWITCH VIDEO**)

- Shall design an animated fireworks MPEG video and
write a Python script that will play the video when
activated by the switch.

This fireworks video was created, and won second place in a separate competition for most creative fireworks video. It was judged based on creativity and school spirit. 

### **Chassis:**
- Chassis will be designed with an aluminum frame. Alu-
minum is abundantly available under the earth’s surface
and mining can be offset with post-mining rehabilitation
and efficient recycling. This constraint will lessen the
broader impact the team has on the environment.

Aluminum is also a light and rigid metal, so it is a good
material to use for a rigid chassis frame. 
