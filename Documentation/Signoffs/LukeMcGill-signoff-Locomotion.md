# Locomotion Subsystem
## Function of the Subsystem
- Transport the robot at a maximum of 0.2032 m/s

- Ensure that the predetermined path can be traversed in the allotted time of three minutes

- The top level controller communicates with the low level controller through USB. The low controller will then send a PWM signal to the motor driver that will control the motor's speed. The low level controller will also send a digital signal to the motor driver that will control the direction of the motor.

## Constraints
- Weight of the robot ( $\approx$ 12 kg)
- Maximum travel speed required (0.2032 m/s)
- Motor torque required (0.0715 Nm per motor)

### Socioeconomic Consideration
- Motor chosen will operate at $\approx$ 90% of it's maximum efficiency. This will help conserve power and reduce battery charging costs.

## Buildable Schematics

### Motor
![image](https://user-images.githubusercontent.com/112428353/203175771-01a94bf9-d55d-4ecd-9e12-75bcc7caf23c.png)

### Electrical Schematic
![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/circuit_schematic.jpg)

### Locomotion Assembly
![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/assembly_side_view.jpg)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/assembly_top_view.jpg)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/assembly_whole_view.jpg)

The CAD model for the motor and wheel assembly can be found here:
https://github.com/nathan-gardner/CapstoneRepo/tree/LukeMcGill-signoff-Locomotion/Documentation/3D%20Models/LocomotionSystem


## Analysis
### Acceleration
$v = 0.2023\ m/s$ (worst case)

$a = \frac{v - v_{o}}{t}$

$a = \frac{0.2023 - 0}{1}$

$a = 0.2032\ m/s^2$

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

$ma_x = F - (\mu_{s}F_n)$

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

The chosen motor is the Pololu #4865. The required motor torque is 7.291 kg-mm. This motor has a max efficiency at 8.6 kg-mm of torque. At 7.291 kg-mm of torque, the motor will operate at $\approx$ 90% of it's maximum efficiency. This motor choice will allow the torque requirments to be met while simultaneously achieving high efficiency. The motor draws 0.28 A. The L298N motor driver supplies 2 A which is more than enough for this motor.

## BOM
| Name of Item   | Description                                                                                 | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
|----------------|---------------------------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|--------|
| Mecanum Wheel  | Mecanum wheels will allow the robot to move and turn in any direction (pack of four wheels) | Locomotion                 | 14209       | ozrobotics       | 1        | 41.48      | 41.48  |
| Wheel Coupling | The wheel couping attaches the wheel to the motor                                           | Locomotion                 | 18077       | ozrobotics       | 4        | 2.34       | 9.36   |
| Motor          | The motors will drive the wheels allowing the robot to move                                 | Locomotion                 | 4865        | Pololu           | 4        | 49.95      | 199.8  |
| Motor Mount    | The motor mount will secure the motor to the chassis (pack of two mounts)                   | Locomotion                 | 2676        | Pololu           | 2        | 7.95       | 15.9   |
| Motor Driver   | The motor drivers supply the voltage and current requirements of the motor                  | Locomotion                 | L298N       | ST               | 2        | 7.41       | 14.82  |
| Total          |                                                                                             |                            |             | Total Components | 13       | Total Cost | 281.36 |

