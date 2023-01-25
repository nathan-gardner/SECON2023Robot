# Top-level Controller Subsystem Signoff

The top-level controller for the robot is the controller that is used as the master for the ROS network on the robot. All other controllers besides the top-level will be Arduino or some other microcontroller, and not have a dedicated full operating system.

## Brief overview of the controller network as a whole

The top-level controller will be connected via USB to the two low-level controllers, which will both be an Arduino Mega 2560. These two controllers where designed in a previous signoff, one of which is dedicated to locomotion, feeding, fireworks, and consumption. The other low level controller is responsible for sorting and storage of the objects around the arena. 

The first low-level controller will have two peripheral Arduino Nano for flipping the switch for the fireworks and the other for controller the actuator that will place the food chips in the correct feeding area. 

It will also send the control signals to the motors for the arena locomotion system. It will send PWM signals to the motor drivers to the control its position in the arena. 

The second microcontroller will be used for the sorting and storage subsystem for the pedestals and ducks for the competition. This will include reading a color sensor for which will in turn actuate a linear actuator that will push pedestals off the conveyor and will allow the ducks to pass, and read a proximity sensor which will initiate a sequence that will drop off a stack of pedestals, a statue, onto the playing field. 

## Function of the Top-level Controller Subsystem

The top-level controller will serve as the master the two aforementioned Arduino Mega controllers, and will serve as the hub for the vision subsystem of the robot. The vision subsystem, which will be a network of sensors, will interface with the top-level controller. The vision subsystem is yet to be designed for this project. 

The top-level controller will host the dedicated OS which is required to orchestrate robot operating system (ROS) connections by serving as the master and providing name registration and lookup for the computation graph. The top-level controller will also have a parameter server which is like a large C struct that will be updated by the nodes when values change in the system. 

Devices, namely the two low level controllers, will host nodes that are dedicated processes for sensors or functions of the robot. This is intended to allow for clean separation of code and to allow for scalability of sensor networks and communication for the robot. 

## Constraints

The top-level controller must have two USB type A ports that will be used to connect the two low-level controllers. The controller must also communicate with the vision sensor network and retrieve data, process that data, and send commands to actuators around the robot, which will perform actions in the arena.

The controller has a size constraint as does the robot. The top-level controller cannot get too hot and cannot require a large fan to cool, because of the power and space constraints. 

In summary, these constraints are as follows:
- Power
- Size
- GPIO
- Computation speed and minimum system specification requirements

## Electrical Schematic

![image](https://user-images.githubusercontent.com/112424739/213819382-0b3b04ac-6564-4466-a17c-074a57101b0d.png)

## Analysis

The top-level controller will need to be analyzed in terms of the constraints: power, size, and GPIO. 

The actual power consumption of a fully equipped Nano running at maximum clock speed sits between 15 - 25 Watts. To power the Nano along with the necessary peripherals through the Micro USB or USB C port, using a 5V adapter that can output 2.5 Amps or higher. If the team uses a 5V DC barrel jack adapter, the outputs must be at least 4 Amps.

The size of Nvidia Jetson Nano Developer Kit 100 x 80 x 29 mm. The team compared this size to the Arduino Mega 2560, which is 53.3 x 101.52. This is a reasonable footprint and the Jetson Nano Dev Kit has mounting holes so that it can be mounted to the robot. 

The total number of GPIO pin headers is 40. The spreadsheet with the GPIO breakdown is below: 

| Type                  | Count |
|-----------------------|-------|
| USB 3                 | 4     |
| Ethernet              | 1     |
| UART                  | 2     |
| SPI                   | 2     |
| HDMI Output Port      | 1     |
| DisplayPort Connector | 1     |
| I2C                   | 2     |
| I2S                   | 1     |
| PWM                   | 2     |
| AUDIO_MCLK            | 1     |
| LCD_TE                | 1     |
| CAM_AF_EN             | 1     |
| GPIO_PZO              | 1     |
| GPIO_PE6              | 1     |
| LCD_BL_PWM            | 1     |

The computation speed and minimum system specification requirements are discussed below in the software analysis portion of this document. 

## Software Analysis - Processing Capabilities and Possible/Probable Software Solutions

This section covers using ROS as a communication framework, and not using the SLAM packages which are available within ROS. ROS will be used for hardware abstraction, low-level device control, functionality implementation, process-to-process messaging, and package management. ROS, in the way it will be implemented on the top-level controller, uses a [publish/subscribe](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern) communication protocol. 

It has been determined that ROS packages will not be used to produce point maps or to perform localization as it relates to the robots navigation, so ROS is exclusively being used for its hardware abstraction, low-level device control, functionality implementation, and process-to-process messaging libraries. To achieve this, the top-level controller will need to have enough compute/ processing power to perform the tasks required of the master: orchestrating connections (naming and registration services) within the computation graph, and hosting nodes within the computation graph itself. This in itself is not very computationally intensive, and according to the [ROS forums](https://www.instructables.com/Getting-Started-with-ROS-Robotic-Operating-Syste/), requires a dual-core computer clocked at 1.6 GHz and at least 2 GB of RAM. The Nvidia Jetson Nano Developer Kit selected meets these requirements. 

The ROS master will be running on the Nvidia Jetson Nano and ROSSerial will be used to establish ROS topics (connections) between the top-level controller and the low-level controllers. As of creating this sign off, the team has been able to create a ROS master on a personal computer (emulating the top-level controller) and create a ROS node on one of the Arduino Mega 2560. This is a simple demo available [here](https://github.com/nathan-gardner/CapstoneRepo/tree/main/Software/demo/Test_ROSSerial), which established a "chatter" topic and publishes std_msg/Int32 data to the topic which can be read by the emulated top-level controller.

This simple demo was created based on this [article](https://sites.duke.edu/memscapstone/using-rosserial-to-setup-a-ros-node-on-a-teensy/) to demonstrate the viability of the design outlined in this sign-off and for the controller network (top and low-level) as a whole, and to make sure ROS was installed correctly. This method for uploading code to the Arduino uses PlatformIO, which is general purpose for different micro-controllers like Arduino, ESP, STM, etc.. It was determined to be the most versatile option instead of using the ArduinoIDE directly.  

ROS has a large amount of community support and will serve as a useful robotics framework. If ROS is unable to be implemented, we will need to use traditional Arduino programming style and establish a serial connection using PySerial between the top-level controller and the low-level controller. This is what ROSSerial is doing but it has already built the framework to create nodes, establish topics, and to publish and subscribe to those topics.

As an example of this implementation, Nathan wrote a [ROS node](https://github.com/nathan-gardner/CapstoneRepo/blob/main/Software/mega2560/sub_consumption/src/main.cpp) for the consumption subsystem which can control the motor speed remotely from anywhere in the ROS computation graph. This uses message passing outlined above, and is what the team plans on using for the serial communication within the controller network. The code in the ROS node above establishes communication between the top and low level controller through topics, one named /consumption/motorState (published) and the other named /consumption/cmdMotorState (subscribed). /consumption/cmdMotorState can be updated from the top level or any node in the computation graph, and this while invoke the analogWrite function on the Arduino and publish an updated value to the /consumption/motorState topic, which is also visible within the computation graph. 

## BOM

| Name of Item                              | Description                                  | Used in which subsystem(s) | Part Number        | Manufacturer     | Quantity | Price      | Total  |
|-------------------------------------------|----------------------------------------------|----------------------------|--------------------|------------------|----------|------------|--------|
| Nvidia Jetson Nano Developer Kit                       | Top-level controller                         | Top-level controller       | 945-13450-0000-100 | Nvidia           | 1        | 149.99     | 149.99 |
| 16 GB or larger UHS-1 microSD card        | Storage device with the Jetson Nano (256 GB) | Top-level controller       | LMSESXX256G-BNAEU  | Lexar            | 1        | 24.99      | 24.99  |
| 5V DC barrel jack power input cable       | Standard barrel jack connector               | Top-level controller       | B07JGR7JJQ         | SIOCEN           | 1        | 9.99       | 9.99   |
| iClever DK03 Bluetooth Keyboard and Mouse | USB connection with a bluetooth chip         | Top-level controller       | B08KZXLTM6         | iClever          | 1        | 39.99      | 39.99  |
| Total                                     |                                              |                            |                    | Total Components | 4        | Total Cost | 224.96 |
