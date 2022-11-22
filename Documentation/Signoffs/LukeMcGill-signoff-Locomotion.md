# Locomotion Subsystem
## Function of the Subsystem
- Transport the robot
## Constraints
- Weight of the robot
- Travel speed required
- Motor torque required

## Buildable Schematics
### Mecanum Wheels
![image](https://user-images.githubusercontent.com/112428353/203180680-b35ef0c3-77bb-44f2-8acd-951273914abb.png)

### Motor
The CAD model for the motors can be found here:
https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/3D%20Models/25d-metal-gearmotor-34-47-encoder.step

![image](https://user-images.githubusercontent.com/112428353/203175771-01a94bf9-d55d-4ecd-9e12-75bcc7caf23c.png)

### L298N Motor Driver
![image](https://user-images.githubusercontent.com/112428353/203175077-d53a8b7e-b3cd-4a24-98c4-44fec2bd8d40.png)

The motor driver will be connected to a 12V power supply.

![image](https://user-images.githubusercontent.com/112428353/203175321-f5006cdd-4413-49f9-ba61-92b5a5842278.png)

The microcontroller will supply the speed and direction signal to the motor driver. Each motor driver drives two motors.

![image](https://user-images.githubusercontent.com/112428353/203179672-5b30a1bc-0743-4b93-b92e-3bebf214f00a.png)

The motor driver will supply the current and voltage to the motors.


## Analysis
### Acceleration
$v = 0.2023\ m/s$ (worst case of 8 inch/second)

$a = \frac{v - v_{o}}{t}$

$a = \frac{0.2023 - 0}{1}$

$= 0.2032\ m/s^2$

### Force
$m = 0.362\ kg$ (mass capacity of wheel)

$\mu_{s} = 0.95$ (wood on rubber)

$\Sigma F_x = ma$

$F_n = mg$

$\Sigma F_x = F - (\mu_{s}F_n)$

$ma_x = F - (\mu_{s}mg)$

$0.362 * 0.2023 = F - (0.95 * 0.362 * 9.8)$

$F = 3.445\ N$

### Torque
$r = 0.024\ m$

$\tau = rFsin(\theta)$

$\tau = 0.024 * 3.445*sin(90^\degree)$

$\tau = 0.0827\ Nm$ (per motor)

$0.0827\ Nm = 8.43\ kgmm$

$\ $

![image](https://user-images.githubusercontent.com/112428353/203174595-19bb7e9c-7a0c-4a4a-93ec-1e5f7feb3a6f.png)

The chosen motor is the Pololu #4865. The required motor torque is 8.43 kg-mm. The motor has max efficiency at 8.6 kg-mm, so this will be very efficient while meeting the torque requirements. The motor draws 0.28 A. The L298N motor driver supplies 2 A which is more than enough.

## BOM
| Name of Item  | Description                                                                | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
|---------------|----------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|--------|
| Mecanum Wheel | Mecanum wheels will allow the robot to move and turn in any direction      | Locomotion                 | 14209MW48   | ROBOTDIGG        | 4        | 16.75      | 67     |
| Motor         | The motors will drive the wheels allowing the robot to move                | Locomotion                 | 4865        | Polulu           | 4        | 49.95      | 199.8  |
| Motor Driver  | The motor drivers supply the voltage and current requirements of the motor | Locomotion                 | L298N       | ST               | 2        | 7.41       | 14.82  |
| Total         |                                                                            |                            |             | Total Components | 10       | Total Cost | 281.62 |
