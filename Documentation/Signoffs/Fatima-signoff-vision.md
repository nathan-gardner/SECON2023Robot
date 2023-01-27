# Function of the subsystem

The vision subsystem for this robot includes the sensor network that will be used for localization within the arena. The path will be preprogrammed into the robot as a large block of conditionals which will access sensor data from the vision subsystem.

The team chose the Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor and RGB Color Sensor with IR filter and White LED. The sensor senses A distance or color from the environment and communicated with the top-level controller via an I2C bus within the vision subsystem. 

The subsystem uses the top-level controller for processing sensor data and so that the data can be read more rapidly, and publish the data to a ROS topic to communicate to the rest of the nodes within the computation graph. Nodes will be created for data acquisitions from the sensors, and will be decoupled from the logic that is making the robot follow its path through the arena. The path locomotion logic will be another node in the network which will subscribe to the data stream created by the sensor nodes, and produce command data for the low-level controller to create at a set interval which will activate the motors to follows the set command encoded in that data stream.

The team chose to use the TCS34725 Color Sensor to detect the duck pond location, which is also used within the design for the sorting subsystem for distinguishing between ducks and pedestals with are going through the bots conveyor belt system. 

## Function

-	Large and small scale measurements are needed to detect position from distance and color
-	The sensors will communicate with top level microcontroller
    - This communication between the top-level controller to the rest of the system will be within the ROS computation graph, so that the sensor acquisition and production can be decoupled from the actual navigation logic. They would be able to run entirely independent from one another but will only work if they are both running at the same time. 

In vision subsystem, the team is going to use two distinct sensors:

## Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor Breakout: 
![image](https://user-images.githubusercontent.com/112426690/214439141-090c5324-f0ba-4fd1-b031-3bf243f64377.png)

- The chip uses 2.6-3.5 VDC, which can come from the Nvidia Jetson Developer Kits 3.3VDC power pin
- This is a I2C device, it has SCL and SDL and they will be connected to the Nvidia Jetson Dev Kit I2C clock and data line.
- The GPIO pin used as an interrupt to indicate data from the sensor
- SHDN pin is the shutdown pin for the sensor.
    - When the SHDN pin is pulled low then the sensor will be in shutdown mode.
- Vin is connected to the power supply from 3 volt to 5 volt (red wire).
- GND is connected to the common power ground (black wire).
- The digital 3 is connected to the SDA pin to the I2C data SDA pin on the Nvidia Jetson Dev Kit.
- Performance: 
    - The field of view (FOV) is 25 degrees.
    - Max ranging capabilities with 33ms timing budges
    - Offset correction done at 10 cm from sensor
    - Detection rate is considered at 94% minimum

The TCS34725 sensor will be 20 mm x 20 mm and is able to be mounted on the bottom of the robot with light so that the color is illuminated and is easiest for the sensor to read. The time of flight LIDAR sensor is 17.78 mm x 25.4 mm. These sensors are both very negligible in size and compared to the rest of the sensors on the robot.

## RGB Color Sensor with IR filter and White LED:
![image](https://user-images.githubusercontent.com/112426690/214433432-a4f3ab95-68f4-47b0-84d3-601dc85e328e.png)

- The power is from 3.3 volt to 5 volts.
- This is a I2C device, it has SCL (pin 2) and SDL (pin 6) and they will be connected to the Nvidia Jetson Dev Kit.
- VDD (pin 1) connected to the power and GND (pin 3) connected to the ground. 
- The team is going to read the power value to attempt to turn it into a value to the RGB LED. 

# Constraints
 
- The robot time of flight LIDAR sensors need to have be be able to measure between $5.08\ -\ 107.63\ cm$ with an accuracy of $+/-\ XXXX$ distance. This is within the distance range of $5\ -\ 120\ cm$ for the absolute distance. 
  - The closest the robot will need to locate itself in is near the wall at the duck pond, which will be $9"\ -\ \frac{width\ of\ robot}{2}=9" - 5.625" = 3.375" = 8.573\ cm$. This can be assumed because no objects will be within $2\ inches$ of the wall, or $5.08\ cm$.
  - The farthest that the robot will need to locate itself with the ToF LIDAR sensor is the length of the arena minus the width of the robot, which is $48" - \frac{11.25"}{2} = 42.375" = 107.63\ cm$.
- The robot must distinguish between different colors on the color spectrum, specifically the difference between black and blue, so that the robot can localize over the duck pond to deliver the duck trailer precisely.
  - The team will calibrate the color sensor for the desired colors spots in the arena floor, using a reference color chart, or using the sensor and measure the reflectance of each color. 
- Data must be produced by the sensors at a high enough rate for the robot to be able to reach accuracy constraints listed above in the first of the constraints. 
- Sensors must be able to connect to one of the existing controller interfaces, either directly to one of the Arduino Mega2560 controllers or to the top level controller USART, SPI, or I2C. 

# Analysis 

## Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor: 

The team assumes that the distance between the robot and the closest wall is  9” – (0.5x11.25”) = 9” – 5.625” = 3.375”.

The LIDAR sensor that we will be using with accuracy at 120 cm indoors is 3%. The max distance that we will be measured when we are finding the pond is 48" + 7" or 55". 3% of 55" is 1.65" of possible error. This was too much error for the original design from the ME team, so the robot corral is being redesigned so that the length will be adjustable in the case that the ToF LIDAR sensors are not accurate enough to localize precisely into the duck pond accurately. Also, according to messages in the official public communication from the hardware competition chair, only a majority of the duck needs to be in the duck pond in order to get points for that duck within the point definition in the competition rules. This means that we really have an additional half duck length to work with on our accuracy tolerance, which equates to about 1.5 inches, because the ducks measure 3.5"x3"x3". The possible error given the 0.707" and the 1.5" allows for a total of 2.207" allowable error if we do not end up adjusting the length of the corral. The team feels that this tolerance and the adjustable corral allows the ToF LIDAR sensor to meet the accuracy requirements to localize the ducks into the duck pond during the competition.

![image](https://user-images.githubusercontent.com/30758520/214467615-b765040b-1130-4919-b1af-a79c21336fe2.png)

The distance sensor will need to be read at a minimum of 66 ms. This is the minimum amount of time needed to acquire an accurate measurement according to the datasheet. We will come in above this to have a comfortable cushion and not acquire more than 10 samples per second from the ToF laser distance sensor. The data acquired will be published to a ROS topic so that it can be subscribed to by the navigation logic node and can be used to perform localization tasks. 

## RGB Color Sensor with IR filter and White LED:

The TCS34725 color sensor is an RGB (red, green, blue) which is a digital light-to-digital converter, which converts the visible light into a digital signal that the external microcontroller reads. The measurements are taken using a 3 x 4 matrix of sensors that have red, green, blue color filters in front of them. Also, to have good accuracy, the integration times must be long and it can be set to 2.4mS, 24mS, 50mS, 101mS, 154mS or 700mS. 

Electrical Specifications:

$V_{DD} = 3\ V$ 

$I_{DD} = 235\ \mu A \ \ (Active)$ 

$I_{DD} = 65\ \mu A \ \ (Wait)$ 

$I_{DD} = 2.5\ \mu A \ \ (Sleep)$ 

The above voltages and currents will be provided by the Nvidia Jetson Nano from the top-level controller subsystem.

Speed: 

Clock Frequency: $\ \ 0-400kHz$

![image](https://user-images.githubusercontent.com/112428796/203214738-1178d2db-62f4-489b-8cfd-b6a167bece1f.png)

Above is the state machine representation for the sensor circuit showing the times each of the states will take. For the majority of the time, the sensor will be in the states idle, RGCB ADC and RGCB INIT after the startup. Detection will take a maximum of 616.4 ms. This is the most time critical application of the color sensor. The second application of the color sensor is pointed towards the ground, and this sample rate will also be sufficient for this application as well. We need to gain meaningful samples, with the robots maximum speed of $0.2023\ \frac{m}{s}$, the robot samples per distance traveled will be $0.2023\ \frac{m}{s} * 0.6164\ s = 0.125\ m = 125\ mm$. The robot for this reason will need to be slowed down from max speed in order to get fine enough samples when we know that we are indeed above the blue duck pond in order to deliver the corral precisely. The speed levels will be designed into the control encodings for the locomotion system.

# Electrical Schematics 

![image](https://user-images.githubusercontent.com/30758520/215147684-46ca0794-2ac8-4caa-bbe2-cadbdf7d479f.png)

# BOM

| Name of Items   | Description                                                  | Used in which subsystem(s) | Part Number | Manufacturer            | Quantity | Price      | Total  |
|-----------------|--------------------------------------------------------------|----------------------------|-------------|-------------------------|----------|------------|--------|
| Color Sensor    | RGB COLOR SENSOR WITH IR FILTER                              | Vision, Sorting            | TCS34725    | Adafruit Industries LLC | 1        | $7.95      | 7.95   |
| Distance Sensor | Adafruit Time of Flight Micro-LIDAR Distance Sensor Breakout | Vision                     | VL53L0X     | Adafruit                | 1        | $14.95     | 14.95  |
| Total           |                                                              |                            |             | Total Components        | 3        | Total Cost | 45.90  |

