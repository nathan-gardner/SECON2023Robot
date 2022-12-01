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

The microcontroller will supply the PWM speed signal and forward and reverse direction signals to the motor driver. Each motor driver drives two motors.

![image](https://user-images.githubusercontent.com/112428353/203179672-5b30a1bc-0743-4b93-b92e-3bebf214f00a.png)

The motor driver will supply the current and voltage to the motors.


## Analysis
### Acceleration
$v = 0.2023\ m/s$ (worst case of 8 inch/second)

$a = \frac{v - v_{o}}{t}$

$a = \frac{0.2023 - 0}{1}$

$= 0.2032\ m/s^2$

### Force
$W_{wheel} = 56 g_f$ (Weight of the wheel)

$W_{capacity} = 3 kg_f$ (Weight capacity of the wheel)

$W_{coupling} = 13.5 g_f$ (Weight of the wheel coupling)

$W_T = 3.0695 kg_f$ (Total weight)

$M_T = 0.3132 kg_m$ (Total mass)

$\mu_{s} = 0.95$ (wood on rubber)

$\Sigma F_x = ma$

$F_n = mg = W_T$

$\Sigma F_x = F - (\mu_{s}F_n)$

$ma_x = F - (\mu_{s}mg)$

$0.3132 * 0.2023 = F - (0.95 * 3.0695)$

$F = 2.9797\ N$

### Torque
$r = 0.024\ m$

$\tau = rFsin(\theta)$

$\tau = 0.024 * 2.9797*sin(90^\degree)$

$\tau = 0.0715\ Nm$ (per motor)

$0.0715\ Nm = 7.291\ kgmm$

$\ $

![image](https://user-images.githubusercontent.com/112428353/203174595-19bb7e9c-7a0c-4a4a-93ec-1e5f7feb3a6f.png)

The chosen motor is the Pololu #4865. The required motor torque is 7.291 kg-mm. This motor has a max efficiency at 8.6 kg-mm, so this will be efficient while also meeting the torque requirements. The motor draws 0.28 A. The L298N motor driver supplies 2 A which is more than enough.

## BOM
| Name of Item   | Description                                                                                 | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
| -------------- | ------------------------------------------------------------------------------------------- | -------------------------- | ----------- | ---------------- | -------- | ---------- | ------ |
| Mecanum Wheel  | Mecanum wheels will allow the robot to move and turn in any direction (pack of four wheels) | Locomotion                 | 14209       | ozrobotics       | 1        | 41.48      | 41.48  |
| Wheel Coupling | The wheel couping attaches the wheel to the motor                                           | Locomotion                 | 18077       | ozrobotics       | 4        | 2.34       | 9.36   |
| Motor          | The motors will drive the wheels allowing the robot to move                                 | Locomotion                 | 4865        | Pololu           | 4        | 49.95      | 199.8  |
| Motor Mount    | The motor mount will secure the motor to the chassis (pack of two mounts)                   | Locomotion                 | 2627        | Pololu           | 2        | 7.95       | 15.9   |
| Motor Driver   | The motor drivers supply the voltage and current requirements of the motor                  | Locomotion                 | L298N       | ST               | 2        | 7.41       | 14.82  |
| Total          |                                                                                             |                            |             | Total Components | 13       | Total Cost | 281.36 |
