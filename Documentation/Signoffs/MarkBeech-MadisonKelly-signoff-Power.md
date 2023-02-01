# **Power Subsystem**
## **Function of The Subsystem** 
- Provides power to all other subsystems that require voltages and currents not supplied by the low-level controllers or the top level controller.
- Allows the robot to be turned off and on.
- Addition of an emergency stop button in order to meet safety standard.
## **Constraints**
- Like all other subsystems, this subsystem is constrained by the size constraint of one cubic foot the robot must fit within at the beginning of the competition. To abide by this constraint, all the necessary components will be chosen with this constraint in mind and placed within the chassis in a strategic location as to save as much space as possible.
- The maximum voltage is the biggest constraint that the robot’s power supply needs to accommodate. The largest voltage any of the robot’s components will need will be 12 V, and the power supply chosen is able to supply 12 V at 6000 mAH. The maximum current draw is 3 A, and this is much more than we will need for any component included in the design. The power supply was also chosen due to its small size, light weight, and rechargeable feature.
- To address the overcurrent concern, the addition of 3A fuses will be made. Since the power supply is only able to supply 3A of current, these 3A fuses will protect the power supply from overcurrent damage.
- Shown below is the voltage and current requirements for each component in each subsystem. These will be the main constraints for the power subsystem.

| Power Subsystem Required Voltage and Current from each Subsystem |                            |             |             |
|----------------------------------------------------------------|----------------------------|-------------|-------------|
| Subsystem                                                      | Component                  | Voltage (V) | Current (A) |
| Locomotion                                                     | Motor Drivers - Motors (2) | 12          | 0.3         |
| Consumption                                                    | Motor Driver - Motor       | 6           | 2.5         |
| Duck Storage and Delivery                                      | Solenoid Actuator          | 12          | 0.65        |
| Duck Storage and Delivery, Pedestal Storage and Delivery, and Feeding | Servo Motor Controller     | 6           | 2.5         |
| Sorting                                                        | Solenoid Actuator          | 12          | 0.3         |
| Sorting                                                        | Motor Controller           | 12          | 2           |
| Top-Level Controller                                           | Jetson Nano                | 5           | 2.5         |

### **Constraint from Standard**

- Shall have a self-latching emergency stop push-button that has a positive operation. The button shall not be a graphical representation or a flat switch based on NFPA 79 - 10.7.2.

## **Buildable Schematic**

### **CAD Model for the subsystem:**

### **Electrical Schematics:**

## **Analysis**

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Buck_converter_model.png)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Buck_converter_output_voltage.png)

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Power_supply_and_buck_converter.png)

### **Power Supply** 
The power supply can supply 12 V at 6000 mAH and 5 V at 12000 mAH. Since each competition round is only 3 minutes, this should provide sufficient power for up to 40 rounds between charging. This will help with testing as well as reducing the risk that the robot's power supply will die during the competition.

## **BOM**
| Name of Item              | Description                                        | Used in which subsystem(s) | Part Number   | Manufacturer     | Quantity | Price      | Total |
|---------------------------|----------------------------------------------------|----------------------------|---------------|------------------|----------|------------|-------|
| Rechargeable 12 V Battery | 12 V/6000 mAH Lithium Ion Battery Pack             | Power                      | YB1206000-USB | TalentCell       | 1        | 40.39      | 40.39 |
| Buck Converters           | DC-DC Adjustable Buck Converters 3-40V to 1.5-35V  | Power                      | LM2596        | ATNSINC          | 1        | 15.69      | 15.69 |
| E-Stop Button             | Self-Locking Emergency Stop Button                 | Power                      | HB2-BS542     | MXUTEUK          | 1        | 9.99       | 9.99  |
|                           |                                                    |                            |               |                  |          |            | 0     |
|                           |                                                    |                            |               |                  |          |            | 0     |
| Total                     |                                                    |                            |               | Total Components | 3        | Total Cost | 66.07 |

