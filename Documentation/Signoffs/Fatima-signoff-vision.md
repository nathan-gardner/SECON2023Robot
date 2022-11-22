# Function of the subsystem 

The robot need to move in a predetermined path from the start point to the end point to collect all objects on the path, if the robot reached the end point, the robot must detect the colors of the spots on the arena ground and move toward the desired spot. Such as the duck pond is the blue spot, when the color have been identified, the robot must go to the desired spot or the duck pond. The robot must identify eight desired spots on the arena. each desired spot has diferent color and the robot will move toward each spot after identifing them. There will be Infrared Sensor and color sensor pluse the camera that will help the robot to do the task.

## Function

- The robot Moving in a predetermined path from the start.
- While it moving in predetermined path without stopping, the objects are entering into the robot mouth.
- The robot arrive the end point and stopped.
- The robot identfying and detecting the desired spots using Camera, infrared Sensor, and color sensor.
- move to all desired spots and release the desired objects. 

# Constraints

# Electrical schematic

![image](https://user-images.githubusercontent.com/112426690/203201208-e5a9c9f0-a94c-40f2-a33d-d8b2f42cc829.jpeg)

# Analysis 

## Sensor Resolution: 
The resolution is the number of pixels present in the camera sensor; the team selects the appropriate resolution using the following simple formula: 

<img width="147" alt="image" src="https://user-images.githubusercontent.com/112426690/203200930-eb0476dd-e64b-4e4f-b0f6-2fafe4743d8d.png">

Where the field of view is the arena which has 4 x 8 feet (where the x-axis is 8 feet x 12) and the smallest feature is the size of the smallest spot in the arena which is the inner circle of radius 2. This calculation is considered the worst case:
Resolution = 2 ((8 x 12 )/2)= 48 pixels , the wanted minimal sensor decision is forty eight pixels.  
A camera with a resolution of 3280x2464 will work because 48 is much less than the smallest dimension which is 2464.

<img width="389" alt="image" src="https://user-images.githubusercontent.com/112426690/203201133-8d6a9e50-2918-44fe-aebc-3fe7c1250a9f.png">

## Sensor size: 
The camera has a pixel size of 1.12µm x1.12µm and a resolution of 3280x2464 pixels. 
The sensor size is then 1.12µm x 3280 by 1.12µm x 2464 = 3.6736 x 2.7596 mm 

#Buildable 

#BOM

