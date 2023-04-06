This document should be a pdf that details all experiments which were conducted. The purpose of the experiment, a description of the experiment, the expected result of the experiment, and the number of trials (must be enough to establish statistically significant results) must be given for each of the experiments.

The document should be in IEEE format.

The results of the experiments should be in the proper format. Data that is best suited to graphs must be in a graph. The same is true of tables. Results should not be given in paragraph format. 


# **Experimental Analysis**

## **Constraints from Signoffs:**

### **Feeding:**
- The design has been changed to two separate cups with two separate servo motors in order to simplify the design. The cups are only big enough to hold the chips, and are mounted on the servo motors which are mounted on the outer edge of the robot, thus saving much needed space inside the robot.(****PUT PICTURE BELOW***)
- Since the design has been simplified in order to save time, the servos do not need to have as much torque. However, they have much more than sufficient torque needed to flip the cups and dump the chips.

### **Locomotion:**
- **Weigh the robot**
- **Need to see how fast the robot can go**
- The motors have enough torque to move the robot. However, there have been instances of browning out in the event that the robot hits the walls of the playing field. To fix this, the team plans to make portions of the robot lighter as well as having fail-safe coding in order to ensure there are no brown outs in the competition.

### **Duck Storage and Delivery:**
- The trailer extends from the back of the robot at the beginning of each run
- The trailer is big enough to hold all ten ducks. (**ADD PHOTO**)
- The team found that the servos not only had enough torque to roll the trailer off the back of the robot, but to also hold the trailer in place during the run.
- The solenoids’ function has now been exchanged in order to allow the robot to straighten itself out by backing up against the wall. In the opposite orientation, the solenoids prevent the trailer to go back onto the robot and back-drive the servo motors.
- The team has found that the gears have no issues staying “meshed” with the rack, so there was no need for the 3D printed enclosure.
- The steel ball transfer used to allow the trailer to roll behind the robot was not able to be mounted in the same way as shown in the signoff due to the wheel placements changing. It now sits up much higher, so the team attached some plastic sheeting on the back of the trailer in order to keep the ducks from “escaping” the trailer.(***ADD PHOTO***)
- Due to shipping issues and sensors not coming in, the team changed the path plan, so the original idea for making a full 180 turn is not necessary. Therefore, that constraint is obsolete.
- The rack and pinion is located inside the robot. Therefore, it is not a pinching hazard.

### **Power:**
- The battery is sufficient in providing 12 V and over 2 A to meet the robot’s needs.
- The output of the battery is regulated by the DC-DC Converter and provides a constant 11.97 V to sufficiently power all components.
- Fuses were not needed in the final design, but were replaced with a current limiting circuit on the locomotion motors in order to ensure the components do not experience a brown out.
- The 6 V loads were delivered via a buck converter. As you can see below, there is a photo of both the buck converters as well as the filtering circuits in order to filter the ripple voltage on the output. (**ADD PICS OF RIPPLE VOLTAGE BEING GONE**)
- The inductive load from the motors was not found to be an issue, so the team decided to not add the capacitors on the input of the motors.
- The team is using a 12 V bus and a 6 V bus. There was no need for a 6 V bus.
- The team was having issues with the Jetson giving an undercurrent message from the USB output despite the analysis showing that it would be enough. The decision was made to implement another buck converter in order to deliver 5.25 V to supply the Jetson.

### **Low-Level Controller:**
- The arduinos had more than enough GPIO especially with all of the sensors not being shipped in and able to be used. The team was able to cut down to using only one arduino instead of the expected two arduinos.

### **Top-Level Controller:**
- The top-level controller is being fed about 5.23 V (have picture) from the battery via the barrel jack.
- **NEEDS TO HAVE SOMETHING HERE ABOUT THE COMPUTATION SPEED**

### **Consumption:**
- Due to current limitations, the team has decided to switch to use a spare locomotion motor for picking up the items. It has enough torque to pick up each item, even multiples at once (**ADD VIDEO**)
- Many variations of spokes have been tested in order to see which ones work the best, and the following spokes were chosen. (***ADD PICS AS WELL AS TALK ABOUT EACH OF THEM**)
- The use of the locomotion motor has been proven sufficient to pick up multiple items, but at a much slower speed than the expected 1.1 second per item. (**TALK ABOUT HOW LONG IT ACTUALLY TAKES**)
- **HOW BIG IS THE CONSUMPTION**. The consumption mechanism is large enough for a duck to be consumed as well as the pedestals.
- As mentioned previously, the path of the robot has changed due to the vision subsystem not being able to be implemented. Due to this, the time to intake items has increased slightly
- There are three walls surrounding the intake in order to protect anyone from moving parts.

### **Fireworks:**
- The robot has the force necessary to flip the switch without issue (***ADD VIDEO***)

## **Measures of Success from Project Proposal**
- Due to the lack of sensors coming in the mail, the team will be unable to detect and start the round from the LED indicator. However, the team replaced this functionality with a start/stop switch. This start/stop switch is fully functional, as is the emergency stop button on every trial. (***ADD VIDEO***)
- The robot is able to drop the chips on each trial run. (***ADD VIDEO***)
- The robot is able to pick up all 10 ducks and hold them in the duck trailer. However, due to the lack of sensors, the pond has to be estimated using the motor encoders. (***Need to find out how many times the robot can do this out of 10 trials**)
- The robot can flip the switch from right to left ** out of 10 trials. The python script has been written as well as the video has been made as of March 10, 2023.
- Due to the sorting subsystem being cut, the pedestals will no longer be held inside the robot and instead will go to the trailer with the ducks. Nor will they be stacked and placed on the playing field anywhere.
- Since the pedestals are not being sorted at all, they will go with the ducks into the duck pond.
