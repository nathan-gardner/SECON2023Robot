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

### **Power Subsystem Required Voltage and Current from each Subsystem**		


| Subsystem                                  | Component                  | Voltage (V) | Current (A) |
| ------------------------------------------ | -------------------------- | ----------- | ----------- |
| Locomotion                                 | Motor Drivers - Motors (4) | 12          | 1.2         |
| Consumption                                | Motor Driver - Motor       | 6           | 0.5         |
| Duck Storage and Delivery                  | Solenoid Actuator          | 12          | 0.65        |
| Pedestal Storage and Delivery, and Feeding | Servo Motor Controller     | 6           | 1.1         |
| Sorting                                    | Solenoid Actuator          | 12          | 0.3         |
| Sorting                                    | Motor Controller           | 12          | 0.25        |
| TOTAL:                                     |                            |             | 4           |

### **Constraint from Standard**

- Shall have a self-latching emergency stop push-button that has a positive operation. The button shall not be a graphical representation or a flat switch based on NFPA 79 - 10.7.2.

## **Buildable Schematic**

### **CAD Model for the subsystem:**

### **Electrical Schematics:**

![image](https://user-images.githubusercontent.com/112424739/216204941-4b3447d1-fe79-48cf-9022-17a319b6a4c3.png)

## **Analysis**

### **Buck Converters**
![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Buck_converter_model.png)

Above is the LTSpice model for the buck converter board chosen for this subsystem.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Power_supply_and_buck_converter.png)

Above is the schematic model for the power supply connected to one of the buck converters.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Buck_converter_output_voltage.png)

Above is the output voltage for the buck converter connected to the 12 V power supply voltage. The output voltage stays around 6V, which is the required voltage for some of the motors and why we are using a buck converter.

### **Motor simulations**

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Consumption_motor.png)

Above is the spice model for the dc motor used in the consumption subsystem.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Consumption_current.png)

Above is the current draw from the consumption motor. This output and model is consistant with what was analyzed for the consumption subsystem.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Locomotion_motor.png)

Above is the spice model for the dc motor used in the locomotion subsystem.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Locomotion_current.png)

Above is the current draw from the locomotion motor. This is accurate to the current draw described in the locomotion subsystem.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Sorting_motor.png)

Above is the spice model for the dc motor used in the sorting subsystem.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Sorting_current.png)

Above is the current draw from the sorting conveyor motor.

### **Power Supply** 

The power supply's output voltage is not regulated, so a 12 V voltage regulator will be needed. The regulator chosen is a L7812CV. The regulator can take a max input voltage of 35 V and will output up to 1.5 A. This low current output is why we need more than one. The connections for the regulators are shown in the electrical schematic above.

The power supply can supply 12 V at 6000 mAH and 5 V at 12000 mAH. Since each competition round is only 3 minutes, this should provide sufficient power for up to 40 rounds between charging. This will help with testing as well as reducing the risk that the robot's power supply will die during the competition.

The power supply's 5 V, 2 A output will be connected via a USB A to USB A cable to the Nvidia Jetson Nano. 

### **Continuous Use**

| Subsystem  | Component | Voltage (V) | Current (A)  |
| ----------- | -------------------------- | -- | ---- |
| Locomotion  | Motor Drivers - Motors (4) | 12 | 1.2  |
| Consumption | Motor Driver - Motor       | 6  | 0.5  |
| Sorting     | Motor Controller           | 12 | 0.25 |
| TOTAL:      |                            |    | 1.95 |

The table above shows each component that will be running continuously from the 12 V supply. These components will need a total of about 2 A in order to function properly. The power supply can supply up to 3 A, which allows for about an extra 1 A of wiggle room for the other components that may switch on for a short period of time.

## **BOM**
| Name of Item              | Description                                                     | Used in which subsystem(s) | Part Number   | Manufacturer     | Quantity | Price      | Total |
|---------------------------|-----------------------------------------------------------------|----------------------------|---------------|------------------|----------|------------|-------|
| Rechargeable 12 V Battery | 12 V/6000 mAH Lithium Ion Battery Pack                          | Power                      | YB1206000-USB | TalentCell       | 1        | 39.99      | 39.99 |
| Buck Converters           | DC-DC Adjustable Buck Converters 3-40V to 1.5-35V               | Power                      | LM2596        | ATNSINC          | 1        | 15.69      | 15.69 |
| E-Stop Button             | Self-Locking Emergency Stop Button 2 NC Red Mushroom 660 V 10 A | Power                      | HB2-BS544     | MXUTEUK          | 1        | 10.99      | 10.99 |
| 3 A Fuses                 | 3A 250V Fuses (pack of 20)                                      | Power                      | F3AL250V      | BOJACK           | 2        | 5.99       | 11.98 |
| Power Distribution Bus    | Solid Brass                                                     | Power                      | 737           | Adafruit         | 2        | 1.95       | 3.9   |
| Total                     |                                                                 |                            |               | Total Components | 7        | Total Cost | 82.55 |


