# **SECON2023Robot - IEEE SoutheastCon Robotics Competition**

## **Final CAD Model for Competition Robot**

![image](/Documentation/Images/finalcadmodel.png)

## **Executive Summary**

The purpose of this project is to build a robot that will be sent to the SoutheastCon student competition in Orlando, Florida. This robot will represent Tennessee Technological University to schools around the Southeastern United States. This document will include further details on the team’s design for the project.

The autonomous tasks the robot may attempt include the following: Feed manatees and alligators the correct food chips, relocate ducks to the duck pond, rebuild pedestals into statues, deposit any unused objects into the recycle bin, and play an animated fireworks video at the end of the round.

The robot was rescoped throughout the year and just before the competition to be able to meet objective and once we know opponents capabilities at the competition. The final robot implemented was able to feed manatees and alligators the correct food chips, relocate ducks to the duck pond, and to push remaining items to the recycling areas. 

The team placed fourth out of thirty six schools in the competition, which is the second best performance in Tennessee Tech history. The team sees this as a success considering the ordering setbacks experienced by all the Capstone teams during the Spring 2023 semester, and the scope changes that were necessary to meet deadlines for the competition. 

Full rules document available [here](/Documentation/IEEE-SoutheastCon-2023-Hardware-Competition-Rules-v3.0.pdf).

## **Capabilities**

The [signoffs](/Documentation/Signoffs/) include function of the subsystem, constraints, buildable schematics, analysis, and bill of materials.

Capabilities of this project are described in signoffs with clear indication of what subsystem progressed through the research phase, which made it into development, and which subsystem were implemented into the robot which went to competition.

The path that this robot follows is defined in [top_level.py](/Software/top-level/top_level_package/scripts/top_level.py) and is all relative to the starting area. The distance is measured using encoder clicks and directions are published within the ROS computation graph. This means the robot cannot observe its surroundings and react to its surroundings, but can only go point to point based on the code within that top_level.py file. This was fine for our street sweeper implementation, and was a necessity in design because none of our vision sensors that we ordered came in the mail. This path implementation was reliable across runs. The direction publications were formatted as ROS Twist messages which gave direction and velocity in rotations per minute. The RPM was maintained on the Arduino using a custom PID controller written in C++. 

## **Salient Outcomes**

The team believes that one salient outcome of this project is a platform for understanding robot operating system for future SECON competitions at Tennessee Tech, and other robotics projects undertaken by capstone groups at the University. This is specifically referring to the TCP/IP network communication established in this project between the Jetson Nano and Arduino Mega2560. 

This project was the closest ever integration of the Mechanical Engineering Department's Senior Design course and the Electrical and Computer Engineering Department's Capstone course. These courses are based around an all-encompassing project from what students have learned in University. The team and its advisors has learned a lot about what has worked and what can be improved with this format of project, and this lessons learned will be taken into the future with the students graduating and for those advisors with future groups.  

## **Project Demonstration & Images**

### **Video of Robot at Competition**

Video of robot at competition during the first round of semi-finals available [here](https://www.youtube.com/watch?v=IqyCZeEZ9IM).

### **Robot image**

![image](/Documentation/Images/robot.png)

More images of the robot available [here](/Documentation/Images/robot/).

#### **Description of Arena:**

- Feeding areas - green and red rectangles in the left corners of the arena
- Starting area - White square in the bottom center of the arena
- Duck Pond - Blue circle in the middle of the top section of the arena
- Recycling areas - Striped white rectangles in the right corners of the arena
- Fireworks switch - Silver rectangle on the right side of the arena
- Inner circles - Three located to the left and right of the starting area, as well as in the middle of the duck pond

#### **Areas utilized for points during competition**

- Feeding areas - Most of our teams points came from correctly delivering the food chips to the feeding areas consistently.  
- Recycling areas - Where the robot pushed loose and randomly placed items to gain points. 
- Duck pond - Ducks ended up in the duck pond during competition. This counted for points on the scoreboard. 

## **Subsystems**
- [Feeding](/Documentation/Signoffs/LukeMcGill-signoff-Feeding.md) - Delivered chips to correct area using servos.
- [Locomotion](/Documentation/Signoffs/LukeMcGill-signoff-Locomotion.md) - Transported robot using mecanum wheels.
- [Power](/Documentation/Signoffs/MarkBeech-MadisonKelly-signoff-Power.md) - Powered every system with two 6V batteries and a 12 V battery.
- [Low-level Controller](/Documentation/Signoffs/NathanGardner-signoff-LowlevelController.md) - Arduino Mega2560 was selected as the main motor controller. 
- [Top-level Controller](/Documentation/Signoffs/NathanGardner-signoff-ToplevelController.md) - Nvidia Jetson selected for computation power as compared to Raspberry Pi.
- [Delivery System](/Documentation/Signoffs/Team2-signoff-ConsumptionSubsystem.md) - pushed objects outward when near a recycling area.

## **Cut Subsystems with explanation**
- [Vision](/Documentation/Signoffs/Fatima-signoff-vision.md) - Sensors were not received in time for building. The team ordered LIDAR sensor and color sensors and they never arrived.
- [Duck Storage](/Documentation/Signoffs/MadisonKelly-signoff-DuckStorage.md) - Subsystem cut as part of the rollback a few days before the competition. We did not have consistency in delivering the ducks stored to the duck pond reliably. A risk reward analysis was done and it was voted on by the team to remove the subsystem entirely. 
- [Pedestal Storage and Delivery](/Documentation/Signoffs/MadisonKelly-signoff-PedestalStorageAndDelivery.md) - Parts were not received in time for building. 
- [Sorting](/Documentation/Signoffs/MarkBeech-signoff-Sorting.md) - Parts were not received in time for building. Sorting was cut when rescoping once we realized we were not receiving parts for many of the subsystems we ordered. 
- [Consumption -> Delivery Subsystem](/Documentation/Signoffs/Team2-signoff-ConsumptionSubsystem.md) - Rescoped 24 hours before the competition. The function was transitioned from consumption into the robot to ejection into the recycling area.  

## **Experimentation**

For each run, there were ten ducks and seven pedestals randomly distributed on the arena as well as six food chips (3 red and 3 green) that the team could pre-load before the run started. The team had three minutes after the start switch was flipped to get as many points as possible. The results from these runs are shown below.

![image](https://user-images.githubusercontent.com/30758520/233695916-4d6cd822-c135-4ab4-8c09-e996e37ffa73.png)

| Round | Green chip | Red chip | Cylinder Recycle | Duck Recycle | Duck Pond | Grand Total Points |
| ----- | ---------- | -------- | ---------------- | ------------ | --------- | ------------------ |
| 1     | 3          | 3        | 1                | 4            | 0         | 52                 |
| 2     | 3          | 3        | 0                | 0            | 0         | 42                 |
| 3     | 2          | 3        | 0                | 0            | 0         | 35                 |
| 4     | 2          | 2        | 4                | 6            | 0         | 48                 |
| 5     | 3          | 3        | 4                | 1            | 0         | 52                 |
| 6     | 2          | 2        | 1                | 2            | 1         | 39                 |

Below is the number of points scored in each round of the competition. If we had integrated the all functionality targeted after the rescope, we would have scored 86 points each round. 

Recycling: $2\ points * 17\ objects = 34\ points$

Feeding: $6\ correct chips * 7\ points = 42 \ points$

Fireworks: $1\ fireworks\ switch\ flip * 10\ points = 10\ points$

Total: $86\ points$

![image](https://user-images.githubusercontent.com/30758520/233694043-275d64be-87a2-4914-8785-920ce69120bd.png)

**Full experimentation document available [here](https://github.com/nathan-gardner/SECON2023Robot/blob/main/Reports/Experimentation.md).**

## **About Us**

### **Team Image**

![image](/Documentation/Images/teamimage.jpg)

**Names from left to right:** Grayson Vermillion, Carlos Salvatierra, Ethan Lewis, Mark Beech, Luke McGill, Nathan Gardner, and Madison Kelly.

### **IEEE Hardware Team**

**ECE team**

Nathan Gardner is an undergraduate Computer Engineering student and is currently a Senior at Tennessee Technological University. Nathan is Team Lead/ Captain for the Southeastcon Robotics Competition Hardware Team. Nathan was responsible for the low-level and top-level controller subsystems and worked on most of the  software implementation for the robot. 

Madison Kelly is an undergraduate Electrical Engineering students and is currently a senior at Tennessee Technological University. Madison is the Project Manager for the Southeastcon Robotics Competition Hardware Team. Madison was responsible for the storage subsystems and was partially responsible for the power subsystem along with Mark Beech.

Fatima Al-Heji is an undergraduate Computer Engineering student and is currently a senior at Tennessee Technological University.

Luke McGill is an undergraduate Electrical Engineering student and is currently a Senior at Tennessee Technological University. Luke was responsible for the locomotion and feeding subsystems. Luke also worked on the software implementation for the robot.

Mark Beech is an undergraduate Electrical Engineering student and is currently a senior at Tennessee Technological University. Mark was responsible for the sorting subsystem and was partially responsible for the power subsystem along with Madison Kelly.

**ME team**

Grayson Vermillion

Carlos Salvatierra is an undergraduate Mechanical Engineering student at Tennessee Technological University. Carlos is a member of the Southeastcon Robotics Competition Hardware Team. Carlos is responsible for creating the robot chassis and drivetrain. Carlos has years of experience working with and leading various high level robotics teams in the past.

Ethan Lewis

### **Faculty Supervisor**

Mr. Jesse Roberts

### **Stakeholders**

The Institute of Electrical and Electronics Engineers (IEEE) is the organization that is hosting the competition in Orlando, Florida on April 13-16th, 2023. Stephen Hopkins is the chair of the hardware competition. Tommy Dillen from Valencia College is the leader of Valencia IEEE Student Chapter design team and designed the arena for the competition.

At the competition, the team will represent the Tennessee Tech ECE Department by showing the skills acquired over most of the ECE curriculum.

Tennessee Technological University's College of Engineering's reputation will be affected by our presence and performance at the competition.

## **Repo Organization**

Below is the layout of the repo.

### **Reports**

Reports are attached in the Reports folder, linked below, and are named "Team2_(NameOfDocument)".

[Reports](/Reports/)

### **Documentation**

The [3D Models](/Documentation/3D%20Models) folder contains any CAD models there might be.

The [Electrical](/Documentation/Electrical) folder contains all electrical schematics.

The [Images](/Documentation/Images) folder contains images of the subsystems.

[Signoffs](/Documentation/Signoffs) folder contains the completed signoffs for each subsystem.

##### **Main Documentation Folder**

[Documentation](/Documentation/)

### **Software**

We will use language and Google (.clang-format) standard programming practices when writing software. We will document our code using Doxygen markdown and commenting style for readability. 

[Software](/Software/)
