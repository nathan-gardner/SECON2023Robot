# **Power Subsystem**
## **Function of The Subsystem** 
- Provides power to all other subsystems that require voltages and currents not supplied indirectly through the low-level controllers or the top level controller pins.
- Allows the robot to be turned off and on.
- Addition of an emergency stop button in order to meet safety standard and competition requirement.
## **Constraints** 
- The power supply must supply 12 V and a current of more than 2 A to accomodate all components that will be running at one time.
- Fuses will need to be added to the power supply bus for added overcurrent protection.
- The table below shows the voltage and current requirements for each component in each subsystem. These will be the main constraints for the power subsystem.
- 6 V will need to be provided to certain components such as some motor drivers and servo drivers.
- An emergency stop button must be used for the safety of those involved in the competition. Further details are shown in the next subsection.


### **Constraint from Standard**

- Shall have a self-latching emergency stop push-button that has a positive operation. The button shall not be a graphical representation or a flat switch based on NFPA 79 - 10.7.2.

### **Power Subsystem Required Voltage and Current from each Subsystem**		


| Subsystem                                  | Component                  | Voltage (V) | Current (A) |
| ------------------------------------------ | -------------------------- | ----------- | ----------- |
| Locomotion                                 | Motor Drivers (2)          | 12          | 1.2         |
| Consumption                                | Motor Driver               | 6           | 0.5         |
| Duck Storage and Delivery                  | Solenoid Actuator          | 12          | 1.17        |
| Pedestal Storage, Duck Storage, Sorting, and Feeding | Servo Motor Controller | 6     | 1.5         |
| Sorting                                    | Motor Controller           | 6           | 0.25        |
| TOTAL:                                     |                            |             | 4.62        |



## **Buildable Schematic**

### **CAD Model for the subsystem:**

Shown below is the layout of the robot. The compact size of this power supply (shown in red) should allow for plenty of room for other components as needed.

![image](https://user-images.githubusercontent.com/112428353/217404755-75b5127b-590a-4b0a-8966-9f2886d97de1.png)


### **Electrical Schematics:**

All electrical circuits will be implemented via soldered breadboard.

![image](https://user-images.githubusercontent.com/112424739/217408012-ff1a457f-138a-45c0-b081-a4b8020c0ec3.png)

## **Analysis**

### **Power Supply** 

The power supply can supply 12 V at 6000 mAH and 5 V at 12000 mAH. Since each competition round is only 3 minutes, this should provide sufficient power for up to 40 rounds between charging. This will help with testing as well as reducing the risk that the robot's power supply will die during the competition.

The power supply's 5 V, 2 A output will be connected via a USB A to USB A cable to the Nvidia Jetson Nano. 

### **Continuous Use Components**

| Subsystem  | Component | Voltage (V) | Current (A)  |
| ----------- | -------------------------- | -- | ---- |
| Locomotion  | Motor Drivers - Motors (4) | 12 | 1.2  |
| Consumption | Motor Driver - Motor       | 6  | 0.5  |
| Sorting     | Motor Controller           | 6  | 0.25 |
| Sorting     | Servo Motor Controller     | 6  | 0.34 |
| TOTAL:      |                            |    | 2.29 |

The table above shows each component that will be running continuously from the 12 V supply. These components will need a total of about 2.3 A in order to function properly. The power supply can supply up to 3 A, which allows for about an extra 0.7 A of wiggle room for the other components that may switch on for a short period of time.

### **Buck Converters**

Buck converter boards will be used to drop the 12 V supply voltage to 6 V for all components needing 6 V to operate. These buck converters are shown on the electrical schematic above.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Buck_converter_model.png)

Above is the LTSpice model for the buck converter board chosen for this subsystem.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Power_supply_and_buck_converter.png)

Above is the schematic model for the power supply connected to one of the buck converters.

![image](https://github.com/nathan-gardner/CapstoneRepo/blob/MarkBeech-MadisonKelly-signoff-Power/Documentation/Images/PowerSubsystem/Buck_converter_output_voltage.png)

Above is the output voltage for the buck converter connected to the 12 V power supply voltage. The output voltage stays around 6V, which is the required voltage for some of the motors and why we are using a buck converter.


### **Emergency stop button**

The emergency stop button will need to have a flyback diode connected to the two output lines of the button to prevent sudden voltage spikes when current is interrupted.

The E-stop button was chosen to be a 2 channel normally closed switch. This was chosen so power can be broken from locomotion, consumption and sorting all together with one button as shown in the electrical schematic above. 


### **Motor simulations**

A $0.1 \ \mu F$ will be placed in parallel with the DC motors in order to reduce RF electromagnetic interference produced from the motor caused by the brushes causing current arcs.

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

## **BOM**
| Name of Item              | Description                                                     | Used in which subsystem(s) | Part Number   | Manufacturer     | Quantity | Price      | Total |
|---------------------------|-----------------------------------------------------------------|----------------------------|---------------|------------------|----------|------------|-------|
| Rechargeable 12 V Battery | 12 V/6000 mAH Lithium Ion Battery Pack                          | Power                      | YB1206000-USB | TalentCell       | 1        | 39.99      | 39.99 |
| Buck Converters           | DC-DC Adjustable Buck Converters 3-40V to 1.5-35V               | Power                      | LM2596        | ATNSINC          | 1        | 15.69      | 15.69 |
| E-Stop Button             | Self-Locking Emergency Stop Button 2 NC Red Mushroom 660 V 10 A | Power                      | HB2-BS544     | MXUTEUK          | 1        | 10.99      | 10.99 |
| 3 A Fuses                 | 3A 250V Fuses (pack of 20)                                      | Power                      | F3AL250V      | BOJACK           | 2        | 5.99       | 11.98 |
| Power Distribution Bus    | Solid Brass                                                     | Power                      | 737           | Adafruit         | 2        | 1.95       | 3.9   |
| Total                     |                                                                 |                            |               | Total Components | 7        | Total Cost | 82.55 |


