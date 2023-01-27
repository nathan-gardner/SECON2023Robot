# Feeding Subsystem

## Function

- Deliver the correct chip to the correct area based on color.

## Constraints

- The size of this subsystem will have to be such that it takes up as minimal space as possible.
- The servo motor must have a torque of at least 75 oz-in.
- The color sensor must be able to detect the red and green animal enclosures.
- The chips must be able to fit within their silos.
- The chip delivery must be on the (R/L??) side of the robot as to allow for a simple delivery to each enclosure.

## Buildable Schematic

## Electrical Schematic
The electrical schematic for the feeding subsystem is shown below. 

![image](https://user-images.githubusercontent.com/112424739/215167750-e4cf1915-8647-4b1a-b4c6-486763774914.png)

## Analysis

### Size

The tubes that the chips will be stored in must be able to fit the chips.

Chip Size $= 1.5 in \ast 0.1 in$
Tube Size $= 1.65 in \ast 0.5 in$

Therefore, the chips will comfortably fit inside the tubes.

### Torque

Weight of the chips $= 0.009 \frac{kg}{chip} \ast 6$ chips = 0.054 kg

$F = m \ast a = 0.054 \ast 9.81 = 0.52974 \frac{N}{m} = 75$ oz-in

The torque of the motor selected is 83 oz-in, so it will be more than sufficient for its cause.

## BOM
| Name of Item           | Description                                        | Used in which subsystem(s)                                        | Part Number | Manufacturer | Quantity | Price | Total |
|------------------------|----------------------------------------------------|-------------------------------------------------------------------|-------------|--------------|----------|-------|-------|
| Servo Motor            | FEETECH Standard Motor 83 oz-in                    | Feeding                                                           | FS5106B     | Pololu       | 1        | 14.95 | 14.95 |
| Servo Motor Controller | Micro Maestro 6-Channel USB Servo Controller       | Feeding, Pedestal Storage and Delivery (included on Pedestal BOM) | 1350        | Pololu       | 1        | 0     | 0     |
| Chip Tube              | 3D printed 1.65 in x 0.5 in tube to hold the chips | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |
| Chip Stopper           | 3D printed plate to stop the chips from falling    | Feeding                                                           | N/A         | Print        | 1        | 0     | 0     |


