# Locomotion Subsystem
## Function of the Subsystem
- Transport the robot
## Constraints
- Weight of the robot
- Travel speed required
- Motor torque required

## Buildable Schematic
## Analysis
### Acceleration
$v = 0.2023\ m/s2$ (worst case of 8 inch/second)

$a = \frac{v - v_{o}}{t}$

$a = \frac{0.2023 - 0}{1}$

$= 0.2032\ m/s^2$

### Force
$m = 0.362\ kg$ (mass capacity of wheel)

$\mu_{s} = 0.95$ (wood on rubber)

$\Sigma F_x = ma$

$F_n = mg$

$\Sigma F_x = F - (\mu_{s}F_n)$

$ma_x = F - (\mu_{s}F_n)$

$0.362 * 0.2023 = F - (0.95 * 0.362 * 9.8)$

$F = 3.445\ N$

### Torque
$r = 0.024\ m$

$\tau = rFsin(\theta)$

$\tau = 0.024 * 3.445*sin(90^\degree)$

$\tau = 0.0827\ Nm$

Chosen motor is Pololu #4865. The required motor torque is 8.43 kg-mm. The motor has max efficiency at 8.6 kg-mm, so this will be very efficient. The motor draws 0.28 A. The L298N motor driver supplies 2 A which is more than enough.

## BOM
