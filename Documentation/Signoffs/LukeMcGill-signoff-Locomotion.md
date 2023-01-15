# Locomotion Subsystem
## Function of the Subsystem
- Transport the robot no less than 0.0677 m/s

- Ensure that the predetermined path can be traversed in the allotted time of three minutes

- The top level controller communicates with the low level controller through USB. The low controller will then send a PWM signal to the motor driver that will control the motor's speed. The low level controller will also send a digital signal to the motor driver that will control the direction of the motor.

## Constraints
- Weight of the robot ( $\approx$ 10.376 kg)
- Minimum travel speed required (0.0677 m/s)
- Motor torque required (0.0604 Nm per motor)

### Socioeconomic Consideration
- Motor chosen will operate at $\approx$ 90% of it's maximum efficiency. This will help conserve power and reduce battery charging costs.

## Buildable Schematics

### Motor
![image](https://user-images.githubusercontent.com/112428353/203175771-01a94bf9-d55d-4ecd-9e12-75bcc7caf23c.png)

The two motor encoders will tell the low level microcontroller how fast the motor is driving, as well as the direction. These will be used in the path planning process.

### Electrical Schematic
![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/locomotion_schematic.jpg)

The electrical schematic can be found here:
https://github.com/nathan-gardner/CapstoneRepo/tree/LukeMcGill-signoff-Locomotion/Documentation/Electrical/Schematics/Sources/Locomotion

### Locomotion Assembly
![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/assembly_side_view.jpg)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/locomotion_top_view.png)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/LukeMcGill-signoff-Locomotion/Documentation/Images/LocomotionSystem/locomotion_whole_view.png)

Each motor will have a motor mount. The mount will attach to the motor by two screws. The motor and mount assembly will attach to the chasis by four screws. Please note that the chasis has not been designed yet, so the model does not show the final product, rather how the motor assebmly will be mounted.

The CAD model for the motor and wheel assembly can be found here:
https://github.com/nathan-gardner/CapstoneRepo/tree/LukeMcGill-signoff-Locomotion/Documentation/3D%20Models/LocomotionSystem


## Analysis
### Acceleration

In order to traverse the entire playing field within the three minute time limit, the robot needs to move no slower than 0.0677 m\s. However, to allow for two minutes to complete the other tasks, the fastest velocity needed will be 0.2023 m/s.

$v = 0.2023\ m/s$ 

$a = \frac{v - v_{o}}{t}$

$a = \frac{0.2023 - 0}{1}$

$a = 0.2032\ m/s^2$

### Force
#### Main Robot Weight
$W_{Motors} = 0.606\ kg_f$

$W_{ConsumptionParts} \approx  1.128\ kg_f$

$W_{Controllers} \approx 0.129\ kg_f$

$W_{Bolts} \approx 0.366\ kg_f$

$W_{Walls} \approx 1.669\ kg_f$

$W_{Power Supply} \approx 1\ kg_f$

$W_{Miscelaneous} \approx 4\ kg_f$

$\ $

#### Robot Caboose Weight
$W_{Ducks} = 0.708\ kg_f$

$W_{Pedestals} = 0.1442\ kg_f$

$W_{Walls} \approx 0.626\ kg_f$

$\ $

#### Total Weight

$W_{Total} \approx 10.376\ kg_f$

$M_{Total} \approx 1.0588\ kg_m$

$\ $

$\mu_{s} = 0.95$ (wood on rubber)

$\Sigma F_x = ma$

$F_n = mg = W_T$

$\Sigma F_x = F - (\mu_{s}F_n)$

$ma_x = F - (\mu_{s}F_n)$

$1.0588 * 0.2023 = F - (0.95 * 10.3762)$

$F = 10.07\ N$

### Torque
$r = 0.024\ m$

$\tau_{Total} = rFsin(\theta)$

$\tau_{Total} = 0.024 * 10.07*sin(90^\degree)$

$\tau_{Total} = 0.2417\ Nm$

$\tau_{Motor} = \tau_{Total} / 4 $

$\tau_{Motor} = 0.0604 \ Nm$

$0.0604\ Nm = 6.149\ kgmm$

$\ $

![image](https://user-images.githubusercontent.com/112428353/203174595-19bb7e9c-7a0c-4a4a-93ec-1e5f7feb3a6f.png)

The chosen motor is the Pololu #4865. It is a 12 V medium power motor. The required motor torque is 6.149 kg-mm. This motor has a max efficiency at 8.6 kg-mm of torque. At 6.149 kg-mm of torque, the motor will operate at $\approx$ 90% of it's maximum efficiency. This motor choice will allow the torque requirments to be met while simultaneously achieving high efficiency. The motor draws 0.28 A. The L298N motor driver supplies 2 A which is more than enough for this motor.

## BOM
| Name of Item   | Description                                                                                 | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
|----------------|---------------------------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|--------|
| Mecanum Wheel  | Mecanum wheels will allow the robot to move and turn in any direction (pack of four wheels) | Locomotion                 | 14209       | ozrobotics       | 1        | 41.48      | 41.48  |
| Wheel Coupling | The wheel couping attaches the wheel to the motor                                           | Locomotion                 | 18077       | ozrobotics       | 4        | 2.34       | 9.36   |
| Motor          | The motors will drive the wheels allowing the robot to move                                 | Locomotion                 | 4865        | Pololu           | 4        | 49.95      | 199.8  |
| Motor Mount    | The motor mount will secure the motor to the chassis (pack of two mounts)                   | Locomotion                 | 2676        | Pololu           | 2        | 7.95       | 15.9   |
| Motor Driver   | The motor drivers supply the voltage and current requirements of the motor                  | Locomotion                 | L298N       | ST               | 2        | 7.41       | 14.82  |
| Total          |                                                                                             |                            |             | Total Components | 13       | Total Cost | 281.36 |

