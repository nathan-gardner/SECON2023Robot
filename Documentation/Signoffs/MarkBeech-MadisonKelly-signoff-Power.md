# **Power Subsystem**
## **Function of The Subsystem** 
- Provides power to all other subsystems that require voltages and currents not supplied by the low-level controllers or the top level controller.
## **Constraints**
### **Size**
- Like all other subsystems, this subsystem is constrained by the size constraint of one cubic foot the robot must fit within at the beginning of the competition. To abide by this constraint, all the necessary components will be chosen with this constraint in mind and placed within the chassis in a strategic location as to save as much space as possible.
- The maximum voltage is the biggest constraint that the robot’s power supply needs to accommodate. The largest voltage any of the robot’s components will need will be 12 V, and the power supply chosen is able to supply 12 V at 6000 mAH. The maximum current draw is 3 A, and this is much more than we will need for any component included in the design. The power supply was also chosen due to its small size, light weight, and rechargeable feature.
- Shown below is the voltage and current requirements for each component in each subsystem. These will be the main constraints for the power subsystem.

| Power Subsystem Required Voltage and Current from each Subsystem |                            |             |             |
|----------------------------------------------------------------|----------------------------|-------------|-------------|
| Subsystem                                                      | Component                  | Voltage (V) | Current (A) |
| Locomotion                                                     | Motor Drivers - Motors (4) | 12          | 0.3         |
| Consumption                                                    | Motor Driver - Motor       | 6           | 1.75-2.5    |
| Duck Storage and Delivery                                      | Solenoid Actuator          | 12          | 0.65        |
| Duck Storage and Delivery                                      | Servo Motor Controller     | 6           | 2.5         |
| Sorting                                                        | Solenoid Actuator          | 12          | 0.3         |
| Sorting                                                        | Motor Controller           | 12          | ?           |
| Top-Level Controller                                           | Jetson Nano                | 5           | ?           |

### **Vision** 
- sensors? (x8):
### **Feeding**
- Uhh idk man:
### **Fireworks**
- Servo?:
### **Constraint from Standard**

- Shall have a self-latching emergency stop push-button that has a positive operation. The button shall not be a graphical representation or a flat switch based on NFPA 79 - 10.7.2.

## **Buildable Schematic**

### **CAD Model for the subsystem:**

### **Electrical Schematics:**

## **Analysis**

### **Power Supply** 
The power supply can supply 12 V at 6000 mAH and 5 V at 12000 mAH. Since each competition round is only 3 minutes, this should provide sufficient power for up to 40 rounds between charging. This will help with testing as well as reducing the risk that the robot's power supply will die during the competition.

## **BOM**
