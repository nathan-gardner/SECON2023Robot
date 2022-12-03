# Function of the subsystem 

The robot need to detect specific position in the arena on a near or far scale. To detect position, the sensor network for micro and macro measurements are needed. The subsystem has a separate microprocessor for processing sensor data and so that the main microcontroller unit in the main controller system can read them more readily. The sensors are communicate with top level microcontroller. And the internal communication between the sensor networks and location microcontroller will need to be a very high speed analog signal. 

## Function

-	micro and macro measurements are needed to detect position from distance and color.
-	The sensors will communicate with top level microcontroller.

# Constraints

The team chose the Adjustable Infrared Sensor Switch and TCS34725 Color Sensor, and a camera IMX219-160 8MP Camera with 160° FOV are used in robotics works on the principle of energy conversion. The final output of the sensor is the electrical signal. The sensor senses the signal from the environment and then generates a corresponding electrical signal and sends it to the controller. Sensors communicate with the microcontroller typically over a data network on the PC board. There’s also a bunch of standard but specialized interfaces. The microcontroller has digital input pins that directly accept digital sensor output. To interface analog sensors, they have an inbuilt Analog to Digital Converter(ADC) circuit. IR sensor outputs data on one pin, Data is sent serially. The team need to connect the IR sensor to pins that allow hardware interrupt, so my program needs Input pins with interrupt capability.
The camera IMX219-160 8MP Camera with 160° FOV has resolution 3280 x 2464, pixel size 1.12µm x1.12µm and focal length of 3.15 mm with 8 Megapixels. The camera is used to perform optical inspections and take measurements of the arena. Also, it is designed to collect image data and transfer the data uncompressed for processing. The robot needs to measure distances on the arena before moving and infrared sensor can provide greater details during the measurement by pointing it at different spots on the arena. After the robot receives all the information, it will move from the start point to the end point on the arena. TCS34725 Color Sensor is used and it has an I2C interface and can detect each spot on the arena. It would be positioned to see all desired spots on the arena ground, and it would scan the floor, detect the colors, and generate a signal to send to the main controller. After identifying the spots’ colors, the sensor will guide the robot to the desired spots. The sensor is able to distinguish between distinct colors as well as detect varying shades of the same hue. The robot must detect green, red, blue, white, black, light and dark grey colors which are desired spots for the robot tasks. The team will calibrate the color sensor for the desired colors spots in the arena floor, using a reference color chart, or using the sensor and measure the reflectance of each color. IR Light is used to reduce gray disparities between color, Dark things absorb infrared wavelengths, resulting in consistency, whereas lighter ones cast shadows. The lighting scheme is useful for detecting inconsistencies in color or shadow variations. While the data of the camera and sensors is passing into Analog/Digital convertor, it is the time the frame grabber takes action. The USB frame grabber is chosen to capture digital frames from analog directly to a computer. 

# Electrical schematic

![image](https://user-images.githubusercontent.com/112426690/203215204-39f99983-1e6c-442c-93ef-867bd4c56353.jpeg)

# Analysis 

## Sensor Resolution: 
The resolution is the number of pixels present in the camera sensor; the team selects the appropriate resolution using the following simple formula: 

$Resolution = 2 \ast ( \frac{Field of View}{Smallest Feature})$

Where the field of view is the arena which has 4 x 8 feet (where the x-axis is 8 feet x 12) and the smallest feature, is the size of the smallest spot in the arena which is the inner circle of radius 2. 
This calculation is considered the worst case:
Resolution = $2 \ast (\frac {8 \ast 12 }{2})  = 48 pixels$
the wanted minimal sensor decision is forty eight pixels.  
A camera with a resolution of 3280x2464 will work because 48 is much less than the smallest dimension which is 2464.


<img width="389" alt="image" src="https://user-images.githubusercontent.com/112426690/203201133-8d6a9e50-2918-44fe-aebc-3fe7c1250a9f.png">

## Sensor size: 
The camera has a pixel size of 1.12µm x1.12µm and a resolution of 3280x2464 pixels. 
The sensor size is then 1.12µm x 3280 by 1.12µm x 2464 = 3.6736 x 2.7596 mm 


# Buildable Schematics 

![image](https://user-images.githubusercontent.com/112426690/203221044-db8d1f5c-1036-4ce3-a675-8dc6f646756e.jpeg)


# BOM

| Name of Item          | Description                                                                        | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
|-----------------------|------------------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|--------|
| IMX219-160 8MP Camera | MX219-160 8MP Camera with 160° FOV - Compatible with NVIDIA Jetson Nano/ Xavier NX | Vision                     | 114992263   | Seeedstudio      | 1        | 22.9       | 22.9   |
| DFRobot SEN0019       | Adjustable Infrared Sensor Switch                                                  | Vision                     | SEN0019     | Shopintertex     | 1        | 10.99      | 10.99  |
| Frame Grabber         | USB Video Frame Grabber Digital MPEG1/2                                            | Vision                     | DM300       | Allaboutadapters | 1        | 23         | 23     |
| TCS34725 Color Sensor | RGB Color Sensor with IR filter and White LED - TCS34725                           | Vision                     | 1334        | adafruit         | 1        | 7.95       | 7.95   |
| Total                 |                                                                                    |                            |             | Total Components | 5        | Total Cost | 364.84 |

