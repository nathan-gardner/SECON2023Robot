# Function of the subsystem 

The robot will detect specific position in the arena on a near or far scale. To detect position, the sensor network for micro and macro measurements are needed. The subsystem has a separate microprocessor for processing sensor data and so that the main microcontroller unit in the main controller system can read them more readily. The sensors are communicate with top level microcontroller. And the internal communication between the sensor networks and location microcontroller will be a very high speed analog signal. 

## Function

-	micro and macro measurements are needed to detect position from distance and color.
-	The sensors will communicate with top level microcontroller.

# Constraints

- The team chose the Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor and RGB Color Snesor with IR filter and White LED. 
- The final output of the sensor is the electrical signal. The sensor senses the signal from the environment and then generates a corresponding electrical signal and sends it to the controller. 
- Sensors communicate with the microcontroller typically over a data network on the PC board. 
- There’s also a bunch of standard but specialized interfaces. The microcontroller has digital input pins that directly accept digital sensor output. 
- To interface analog sensors, they have an inbuilt Analog to Digital Converter(ADC) circuit. 
- The robot needs to measure distances on the arena before moving and Distance sensor can provide greater details during the measurement by pointing it at different spots on the arena.
- After the robot receives all the information, it will move from the start point to the end point on the arena. 
- Color Sensor is used and it has an I2C interface and can detect each spot on the arena. It would be positioned to see all desired spots on the arena ground, and it would scan the floor, detect the colors, and generate a signal to send to the main controller. 
- After identifying the spots’ colors, the sensor will guide the robot to the desired spots. The sensor is able to distinguish between distinct colors as well as detect varying shades of the same hue. 
- The team will calibrate the color sensor for the desired colors spots in the arena floor, using a reference color chart, or using the sensor and measure the reflectance of each color. 
- The USB frame grabber is chosen to capture digital frames from analog directly to a computer. 

# Electrical schematic

![image](https://user-images.githubusercontent.com/112426690/206012268-de87b7f5-e74b-4b6b-a0f0-57fd8a6225c6.png)

# Analysis 

## Sensor Resolution: 
??

<img width="389" alt="image" src="https://user-images.githubusercontent.com/112426690/203201133-8d6a9e50-2918-44fe-aebc-3fe7c1250a9f.png">

## Sensor size: 
??

# Buildable Schematics 

![image](https://user-images.githubusercontent.com/112426690/206012248-b4077c5a-735d-4008-9b8c-3614c5440930.png)

# BOM

| Name of Item          | Description                                                                        | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
|-----------------------|------------------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|--------|
| IMX219-160 8MP Camera | MX219-160 8MP Camera with 160° FOV - Compatible with NVIDIA Jetson Nano/ Xavier NX | Vision                     | 114992263   | Seeedstudio      | 1        | 22.9       | 22.9   |
| DFRobot SEN0019       | Adjustable Infrared Sensor Switch                                                  | Vision                     | SEN0019     | Shopintertex     | 1        | 10.99      | 10.99  |
| Frame Grabber         | USB Video Frame Grabber Digital MPEG1/2                                            | Vision                     | DM300       | Allaboutadapters | 1        | 23         | 23     |
| TCS34725 Color Sensor | RGB Color Sensor with IR filter and White LED - TCS34725                           | Vision                     | 1334        | adafruit         | 1        | 7.95       | 7.95   |
| Total                 |                                                                                    |                            |             | Total Components | 4        | Total Cost | 64.84 |

