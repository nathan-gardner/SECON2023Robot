# Team 2 Functional Prototype Substitution Request

## Original Requirementa
### Top Level Controller
- Able to publish ROS topics from command line to the low level controllers
### Low Level controller
- Able to send accurate controls signals to motors
### Pedestal Storage
- Able to get data from the sensors (maybe incorrect data, but of correct data type)
- All sensors need to be connected, but getting data from each sensor
Connecting up the servo is sufficient in the case that Carlos does not have a Physical prototype for the silo
- The servo needs to be operating electrically and have the ability to ge driven by the controller
### Duck Storage
- Able to actuate solenoid locks
- Able to spin servos a full 360 degrees
### Consumption
- Fully assembled
- Spokes spinning
### Locomotion
- Ability to move robot in all directions allowable by the mecanum wheels (forward, backward, left, right and rotating)
### Fireworks
- Nothing
### Sorting
- Able to move conveyor belt
- Able to actuated servo flipper
- Able to actuate servo based on color sensor data
### Feeding
- Servo is able to be be driven by the motor driver commanded by the controller network
### Power
- Able to regulate battery output to 12 V
- Able to supply 6 V via buck converters where necessary
- All components that can be connected together actually connected


## 1. Firework Video
- Substitude the completed firework video for the sorting minimally functional prototype
  - The motor and color sensor parts have not come in yet, so we would like to substitute the animated fireworks video which is automatically run by a Python script on a Raspberry Pi to emulate competition conditions.  

## 2. Motor Encoders
- Substitute working motor encoders for the tof distance sensors
  - The distance sensor parts have not come in yet
- Motor encoders were wired up, and they provide a precise number that shows position of the wheels and can be used to determine distance traveled and will be our feedback signal for the motors. The motor encoders are essential to reading distance traveled and were not initially part of the minimally functional prototype. 

## 3. Prototype Chassis
- Substitute the built proto-chassis for pedestal storage minimally functional expectation
  - Proximity sensor for pedestal storage has not come in yet

## 4. Consumption
- Substitute consumption for duck storage servo operation
  - Consumption will have motor and spokes connected. The consumption subsystem will be ready for experimentation and fully operational. 
  - The servos have not come in yet for duck storage, but the solenoid has and is functional
