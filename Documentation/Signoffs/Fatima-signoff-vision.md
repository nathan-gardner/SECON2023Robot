# Vision Sensor Network Subsystem Signoff

# Function of the subsystem

The vision subsystem for this robot includes the sensor network that will be used for localization within the arena. The path will be preprogrammed into the robot as a large block of conditionals which will access sensor data from the vision subsystem.

The team chose the Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor for distance measurements in the arena and the RGB Color Sensor with IR filter for color sensing within the robot for active sorting and in order to find landmarks within the arena. 

The subsystem uses the top-level controller for processing sensor data, reading data more rapidly, and publishing the data to a ROS topic to communicate to the rest of the nodes within the computation graph. Nodes will be created for data acquisitions from the sensors, and will be decoupled from the logic that is making the robot follow its path through the arena. The path locomotion logic will be another node in the network which will subscribe to the data stream created by the sensor nodes, and produce command data for the low-level controller to create at a set interval which will activate the motors to follows the set command encoded in that data stream.

The team chose to use the TCS34725 Color Sensor to detect the duck pond location, which is also used within the design for the sorting subsystem for distinguishing between ducks and pedestals with are going through the bots conveyor belt system. 

## Function

The vision subsystem is a sensor network that will allow the robot to determine its position in the arena. This is done by finding the distance to the nearest walls, and also using a color sensor pointed towards the ground to detect the feeding areas and the duck pond in the middle of the arena. 

-	Large and small scale measurements are needed to detect position from distance and color
-	The sensors will communicate with top level microcontroller, the color sensor needs to be capable of distinguishing needed colors in the arena: blue, red, green, pink, and yellow. 
    - This communication between the top-level controller to the rest of the controller network will be within the ROS computation graph, so that the sensor acquisition and production can be decoupled from the actual navigation logic. They would be able to run entirely independent from one another but will only work if they are both running at the same time. 

# Constraints

The closest to the wall the robot will need to localize, or project a LIDAR towards a wall to find it general position in the arena, is near the wall at the duck pond. This will be $9"\ -\ \frac{width\ of\ robot}{2}=9" - 5.625" = 3.375" = 8.573\ cm$. This can be assumed because no objects will be within $2"$ of the wall, or $5.08\ cm$. The farthest from the wall that the robot will need to localize within the arena with the ToF LIDAR sensor is the length of the arena minus half the width of the robot, which is $48" - \frac{11.25"}{2} = 42.375" = 107.63\ cm$. This maximum distance will be when the robot is in the center of the arena long ways, for example, in the duck pond. The robot time of flight LIDAR sensors need to have be be able to measure between $5.08\ -\ 107.63\ cm$ with an accuracy of $+/-\ 0.707"$ or $+/-\ 1.8 cm$. This is within the distance range of $5\ -\ 120\ cm$ for the absolute distance, which is specified within the LIDAR sensor data sheet. The accuracy requirement ( $+/-\ 1.8 cm$ ) is analyzed below in the analysis section of this sign off. 

The robot must distinguish between different colors on the color spectrum, specifically the difference between black ( `0x000000` ) and blue ( `0x2876BB` ), so that the robot can localize over the duck pond to deliver the duck trailer precisely. These color values were taking from color sampling the image in the competition rules. 

LIDAR sensor data must be produced by the sensors at a high enough rate for the robot to be able to reach needed accuracy constraints.  

Sensors must be able to connect to one of the existing controller interfaces, either directly to one of the Arduino Mega2560 controllers or to the top level controller USART, SPI, or I2C.

List of constraints:

- Robot LIDAR sensor sensor distance thresholds (50 - 1200 mm)
- Color sensor accuracy, specifically with red, green, blue, and black (responsivity analysis below)
- Data production rate from the sensors, or sensor resolution (color sensor: take samples every $41.7\ mm$, LIDAR: < 400 kbit/sec)
- Sensor communication protocols availability (USART, SPI, I2C)

# Buildable Schematics

## CAD models

Below are images showing how all of the subsystems, including the vision sensor network will fit into the robot.

Top view

![image](https://user-images.githubusercontent.com/30758520/217405647-4aef4118-8f63-4c85-bbfe-5125365fd0a0.png)

Side view

![image](https://user-images.githubusercontent.com/30758520/217405754-b7508cac-6b67-48e5-bc1c-969beabe3828.png)

Rear Side view 1

![image](https://user-images.githubusercontent.com/30758520/217406171-b0454923-5b69-47fe-aef1-dd19d3acfe9c.png)

Rear Side view 2

![image](https://user-images.githubusercontent.com/30758520/217406076-af1fa457-ba10-4bc0-bd10-c326b97fa633.png)

Rear view

![image](https://user-images.githubusercontent.com/30758520/217406232-0fdc2a19-cdb0-42c5-bda1-3b213afa7b6e.png)

Front corner

![image](https://user-images.githubusercontent.com/30758520/217406258-9ac6acc2-2f57-4e2d-a900-f2ad3f18a0f0.png)

Bottom

![image](https://user-images.githubusercontent.com/30758520/217406342-82d7b362-cc9f-40ac-a97d-a80c6cf51ee9.png)

## Electrical Schematics 

![image](https://user-images.githubusercontent.com/30758520/218519001-13fccbc9-3add-4bb9-8bc5-e52de2323b04.png)

The 40-pin header connection design is represented in this sign off, and not in the top-level controller sign off.

# Analysis 

## Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor: 

### Robot LIDAR sensor distance thresholds analysis

Max error calculation based on duck pond location:

The team knows from dimensions given in the specification that the distance between the robot (when in the duck pond) and the closest wall is 

$9” – (0.5x11.25”) = 9” – 5.625” = 3.375”$.

The middle of the duck pond is 9" away from the wall. The robot width is 11.25". Analysis below shows how far the sensor on the side of the robot will be away from the wall when the robot is on the duck pond.

The team knows from dimensions given in the specification that the distance between the robot and the closest wall when in the circle is  

$9” – (0.5 \ast 11.25”) = 9” – 5.625” = 3.375”$. 

Max distance the ToF LIDAR will measure the distance to the furthest wall (distance from the wall to the middle of the circle, minus half the robot width)

$48" - 0.5 \ast 11.25 = 42.375'$

and considering possible 3% error (from datasheet) will be

$\therefore possible\ error = 3$% $of\ 42.375" = 1.27"$

The robot will have ability to measure $42.375" ± 1.27"$ with the ToF LIDAR sensor when positioning itself around the duck pond. 

The above analysis fits into the 50 - 1200 mm (5.08" - 47.24") constraint defined. 

*Note: Corral is being designed to be adjustable on it longest dimension if the corral needs to be smaller in order to land in the duck pond. A majority of the duck needs to be in the duck pond to count for points.*

Below is an image of how the corral will fit into the duck pond when it is fully extended.

![image](https://user-images.githubusercontent.com/30758520/214467615-b765040b-1130-4919-b1af-a79c21336fe2.png)

If considering only this 0.707", this is not sufficient. Because only a majority of each duck needs to be in the pond to count for point allocations. This also gives us another half duck length and makes the allowable error calculation shown below.

$0.707" + 1.5" = 2.207"$ 

This distance of 2.207 in is less than the possible error. Therefore, $42.375" ± 1.27"$ accuracy will be sufficient for the team's needs.

## RGB color sensor with IR filter: 

### Color sensor accuracy, specifically with red, green, blue, and black analysis

Interrupts will be set to generate when the light intensity has exceeded the set threshold. Threshold values depend on many variables that can not be simulated like light allowed in the robot by the chassis, light provided by the sensor, and the distance from the target. 

![image](https://user-images.githubusercontent.com/30758520/218214807-59ad3827-e0c5-40d2-95bf-e212b0e115c6.png)

Above is the photodiode spectral responsivity (optical-to-electrical conversion efficiency) graph. Colors within the arena were purposefully chosen to fall at the peaks of the blue, green, and red curves shown. This makes color measurement easier, because colors should not regularly fall between the responsivity peaks. Color falls within the ranges below, which can be met with the color sensor selected. 

Blue: $450\ nm$

Green: $550\ nm$

Red: $650\ nm$

*Source: ![Encyclopedia Britannica](https://www.britannica.com/science/color/The-visible-spectrum)*

### Color sensor analysis for green versus red

*Red and green are very far from each other on the color spectrum below are calculations showing confidence that the color sensor can distinguish between red and green.*

### Color sensor analysis for black versus blue

*Below are calculations showing confidence that the color sensor can distinguish between blue and black.*

### Data production rate from the sensors, or sensor resolution analysis

$F_{clock}\ \ 0-400kHz$

![image](https://user-images.githubusercontent.com/112428796/203214738-1178d2db-62f4-489b-8cfd-b6a167bece1f.png)

The majority of the time, the sensor will be in the states idle, RGBC ADC and RGBC INIT after the startup. Detection will take a maximum of 616.4 ms.

When the color sensor is pointed towards the ground, the bot will move at $0.2023\ \frac{m}{s}$, the robot samples per distance traveled will be $0.2023\ \frac{m}{s} * 0.6164\ s = 0.125\ m = 125\ mm$. The robot will be slowed down when finer samples are needed. $0.0677 \frac{m}{s}$ is the minimum speed of the robot, and it will be able to take samples every $41.7\ mm$ at that speed. 

### Sensor communication protocols availability (USART, SPI, I2C)

All of these sensor use I2C, which is a very common communication protocol which allows for many devices on a single bus. Both of our controller models support I2C communication. 

# Software Consideration - Possible/Probable Software Solutions

Public software libraries have been created for interfacing with boh sensors selected for the vision sensor network. Libraries are provided by Adafruit and linked below,

![TCS34725 - Color Sensor Software Library](https://github.com/adafruit/Adafruit_TCS34725)

![VL53L0X - ToF LIDAR Sensor Software Library](https://github.com/adafruit/Adafruit_VL53L0X)

# BOM

| Name of Items   | Description                                                  | Used in which subsystem(s) | Part Number      | Manufacturer            | Quantity | Price      | Total   |
|-----------------|--------------------------------------------------------------|----------------------------|------------------|-------------------------|----------|------------|---------|
| Color Sensor    | RGB COLOR SENSOR WITH IR FILTER                              | Vision                     | TCS34725         | Adafruit Industries LLC | 2        | $7.95      | $15.90   |
| Distance Sensor | Adafruit Time of Flight Micro-LIDAR Distance Sensor Breakout | Vision                     | VL53L0X          | Adafruit                | 8        | $14.95     | $119.60  |
| Resistors       | EDGELEC 100pcs 4.7K ohm Resistor 1/2w (0.5Watt) ±1% Tol      | Vision                     | EFR-W0D50-A:MF   | EDGELEC                 | 1        | $5.99      | $5.99   |
|                 |                                                              |                            |                  | Total Components        | 3        | Total Cost | $141.49  |
