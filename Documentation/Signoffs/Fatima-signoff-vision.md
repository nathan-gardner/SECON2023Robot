# Function of the subsystem

The vision subsystem for this robot includes the sensor network that will be used for localization within the arena. This is any sensor that is used for micro- or macro location. The path will be preprogrammed into the robot as a large block of conditionals which will access sensor data from the vision subsystem.

The robot will detect specific position in the arena on a near or far scale. To detect position, the sensor network for micro and macro measurements are needed. The subsystem uses the top-level controller for processing sensor data and so that the low-level controller can read the sensors more rapidly, and publish the data to a ROS topic to communicate with the top-level controller. 

The team chose the Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor and RGB Color Sensor with IR filter and White LED. The final output of the sensor is the electrical signal. The sensor senses A distance from the environment and then generates a corresponding electrical signal and sends it to the controller.

The team chose to use the TCS34725 Color Sensor to detect the duck pond location, which is also used within the design for the sorting subsystem for distinguishing between ducks and pedestals with are going through the bots conveyor belt system. 

## Function

-	Large and small scale measurements are needed to detect position from distance and color
-	The sensors will communicate with top level microcontroller
    - This communication will be within the ROS computation graph, so that the sensor acquisition and production can be decoupled from the actual navigation logic. 

# Constraints
 
- The robot time of flight LIDAR sensors need to have be be able to measure between XXXX - XXXX distance with an accuracy of +/- XXXX distance. 
- The robot must distinguish between different colors on the color spectrum, specifically the difference between black and blue, so that the robot can localize over the duck pond to deliver the duck trailer precisely.
  - The team will calibrate the color sensor for the desired colors spots in the arena floor, using a reference color chart, or using the sensor and measure the reflectance of each color. 
- Data must be produced by the sensors at a high enough rate for the robot to be able to reach accuracy constraints listed above in the first of the constraints. 
- Sensors must be able to connect to one of the existing controller interfaces, either directly to one of the Arduino Mega2560 controllers or to the top level controller USART, SPI, or I2C. 

# Electrical schematic


# Analysis 
In vision subsystem, the team is going to use two sensors:

## RGB Color Snesor with IR filter and White LED:
![image](https://user-images.githubusercontent.com/112426690/214433432-a4f3ab95-68f4-47b0-84d3-601dc85e328e.png)

- The power is from 3.3 volt to 5 volts.
- This is a I2C device, it has SCL (pin 2) and SDL (pin 6) and they will be connected to the arduino (pin A4 and pin A5).
- VDD (pin 1) connected to the power and GND (pin 3) connected to the ground. 
- The team is going to read the power value to attempt to turn it into a value to the RBG LED. 
    - the team might use common cathod and want to put resistors on these pins.
    - the team will use four resistors 1k ohm or 2k ohm and they will be connected to the arduino because we need to output analog values to them.

## Adafruit VL53L0X Time of Flight Micro-LIDAR Distance Sensor Breakout: 
![image](https://user-images.githubusercontent.com/112426690/214439141-090c5324-f0ba-4fd1-b031-3bf243f64377.png)

- The chip uses 2.8 VDC, the team will included a voltage regulator on board that will take 3 VDC to 5 VDC.
- This is a I2C device, it has SCL and SDL and they will be connected to the microcontrollers I2C clock and data line.
- The GPIO pin used to indicate that data.
- SHDN pin is the shutdown pin for the sensor.
    - When the SHDN pin is pulled low then the sensor will be in shutdown mode.
- Vin is connected to the power supply from 3 volt to 5 volt (red wire).
- GND is connected to the common power ground (black wire).
- The digital 3 is connected to the SDA pin to the I2C data SDA pin on the Arduino.


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

