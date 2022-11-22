# Function of the subsystem 

The robot need to move in a predetermined path from the start point to the end point to collect all objects on the path, if the robot reached the end point, the robot must detect the colors of the spots on the arena ground and move toward the desired spot. Such as the duck pond is the blue spot, when the color have been identified, the robot must go to the desired spot or the duck pond. The robot must identify eight desired spots on the arena. each desired spot has diferent color and the robot will move toward each spot after identifing them. There will be Infrared Sensor and color sensor pluse the camera that will help the robot to do the task.

## Function

- The robot Moving in a predetermined path from the start.
- While it moving in predetermined path without stopping, the objects are entering into the robot mouth.
- The robot arrive the end point and stopped.
- The robot identfying and detecting the desired spots using Camera, infrared Sensor, and color sensor.
- move to all desired spots and release the desired objects. 

# Constraints

The team choose the camera IMX219-160 8MP Camera with 160° FOV - Compatible with NVIDIA Jetson Nano/ Xavier NX. Which has resolution 3280 x 2464, pixel size 1.12µm x1.12µm and focal length of 3.15 mm with 8 Megapixels. The camera is used to perform optical inspections and take measurements of the arena. Also, it designed to collect image data and transfer it uncompressed for processing. The robot need to measure distances on the arena before moving and infrared sensor can provide greater details during the measurement by pointing it at different spots on the arena. After the robot received all the information, it will move from the start point to the end point on the arena. TCS34725 Color Sensor is used and it has an I2C interface and can detect each spot on the arena. And it would be positioned to see all desired spots on the arena ground, then it would scan the floor and detect the colors and generate a signal. After identifying the spots’ colors, the sensor will guide a robot to the desired spots. The sensor is able to distinguish between distinct colors as well as detect varying shades of the same hue. The robot must detect green, red, blue, white, black, light and dark grey colors which are desired spots for the robot tasks. The team will calibrate color sensor for the desired colors spots in the arena floor, using a reference color chart or using the sensor and measure the reflectance of each color. IR Light is used to reduce gray disparities between color, Dark things absorb infrared wavelengths, resulting in consistency, whereas lighter ones cast shadows. The lighting scheme is useful for detecting inconsistencies in color or shadow variations. And its going to the camera. While the data of the camera and sensors is passing into Analog/Digital convertor, it is the time the frame grabber takes action. The USB frame grabber is chosen to captures digital frames from analog directly to a computer. The Processor that will be used in robotic vision systems is  NVIDIA® Jetson™ processor family Specifically, Jetson TX2 series because It is the most power embedded AI computer device available. It features a number of common hardware connectors, allowing it to be readily incorporated into a variety of devices and form factors.

# Electrical schematic

![image](https://user-images.githubusercontent.com/112426690/203215204-39f99983-1e6c-442c-93ef-867bd4c56353.jpeg)

# Analysis 

## Sensor Resolution: 
The resolution is the number of pixels present in the camera sensor; the team selects the appropriate resolution using the following simple formula: 

$Resolution = 2 \ast ( \frac{Field of View}{Smallest Feature})$

Where the field of view is the arena which has 4 x 8 feet (where the x-axis is 8 feet x 12) and the smallest feature, is the size of the smallest spot in the arena which is the inner circle of radius 2. 
This calculation is considered the worst case:
Resolution = $2 \ast ((8 \ast 12 )\frac  2 ) = 48 pixels$
the wanted minimal sensor decision is forty eight pixels.  
A camera with a resolution of 3280x2464 will work because 48 is much less than the smallest dimension which is 2464.


<img width="389" alt="image" src="https://user-images.githubusercontent.com/112426690/203201133-8d6a9e50-2918-44fe-aebc-3fe7c1250a9f.png">

## Sensor size: 
The camera has a pixel size of 1.12µm x1.12µm and a resolution of 3280x2464 pixels. 
The sensor size is then 1.12µm x 3280 by 1.12µm x 2464 = 3.6736 x 2.7596 mm 


# Buildable 

![image](https://user-images.githubusercontent.com/112426690/203221044-db8d1f5c-1036-4ce3-a675-8dc6f646756e.jpeg)


# BOM

| Name of Item          | Description                                                                        | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
|-----------------------|------------------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|--------|
| IMX219-160 8MP Camera | MX219-160 8MP Camera with 160° FOV - Compatible with NVIDIA Jetson Nano/ Xavier NX | Vision                     | 114992263   | Seeedstudio      | 1        | 22.9       | 22.9   |
| DFRobot SEN0019       | Adjustable Infrared Sensor Switch                                                  | Vision                     | SEN0019     | Shopintertex     | 1        | 10.99      | 10.99  |
| Frame Grabber         | USB Video Frame Grabber Digital MPEG1/2                                            | Vision                     | DM300       | Allaboutadapters | 1        | 23         | 23     |
| TCS34725 Color Sensor | RGB Color Sensor with IR filter and White LED - TCS34725                           | Vision                     | 1334        | adafruit         | 1        | 7.95       | 7.95   |
| NVIDIA® Jetson™ TX2   | NVIDIA® Jetson™ processor family                                                   | Vision                     | 9.00136E+14 | NVIDIA®          | 1        | 300        | 300    |
| Total                 |                                                                                    |                            |             | Total Components | 5        | Total Cost | 364.84 |

