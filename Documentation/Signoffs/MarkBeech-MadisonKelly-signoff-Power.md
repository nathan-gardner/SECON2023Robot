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
- The ripple voltage from the inductive load of the motors should be eliminated in order to protect other components. To fix this, the team will add smoothing capacitors on each motor load.
- An emergency stop button must be used for the safety of those involved in the competition. Further details are shown in the next subsection.


### **Constraint from Standard**

- Shall have a self-latching emergency stop push-button that has a positive operation. The button shall not be a graphical representation or a flat switch based on NFPA 79 - 10.7.2.

### **Power Subsystem Required Voltage and Current from each Subsystem**		


| Subsystem                                  | Component                  | Voltage (V) | Current (A) |
| ------------------------------------------ | -------------------------- | ----------- | ----------- |
| Locomotion                                 | Motor Drivers (2)          | 12          | 1.2         |
| Consumption                                | Motor Driver               | 6           | 0.5         |
| Duck Storage and Delivery                  | Solenoid Actuator          | 12          | 0.65        |
| Pedestal Storage, Duck Storage, Sorting, and Feeding | Servo Motor Controller | 6     | 2.17        |
| Sorting                                    | Motor Controller           | 6           | 0.25        |
| TOTAL:                                     |                            |             | 4.77        |



## **Buildable Schematic**

### **CAD Model for the subsystem:**

Shown below is the layout of the robot. The compact size of this power supply (shown in red) should allow for plenty of room for other components as needed.

![image](https://user-images.githubusercontent.com/112428353/217404755-75b5127b-590a-4b0a-8966-9f2886d97de1.png)


### **Electrical Schematics:**

All electrical circuits will be implemented via soldered breadboard.

![image](https://user-images.githubusercontent.com/112424739/217408012-ff1a457f-138a-45c0-b081-a4b8020c0ec3.png)

## **Analysis**

### **Power Supply** 

The power supply can supply 12 V at 6000 mAH and 5 V at 12000 mAH and the maximum current output is 3A. Since each competition round is only 3 minutes, the following calculations were performed. 

$Time = \frac{6000 \frac{mA}{H}}{3A} = 2 hours = 120mins$

$rounds = \frac{120}{3} = 40 rounds$

The robot should be able to run up to 40 rounds without needing a charge. This will help with testing as well as reducing the risk that the robot's power supply will die during the competition.

The power supply's 5 V, 2 A output will be connected via a USB A to USB A cable to the Nvidia Jetson Nano. 

### **Continuous Use Components**

| Subsystem  | Component | Voltage (V) | Current (A)  |
| ----------- | -------------------------- | -- | ---- |
| Locomotion  | Motor Drivers - Motors (4) | 12 | 1.2  |
| Consumption | Motor Driver - Motor       | 6  | 0.5  |
| Sorting     | Motor Controller           | 6  | 0.25 |
| TOTAL:      |                            |    | 1.95 |

The table above shows each component that will be running continuously from the 12 V supply. These components will need a total of about 2 A in order to function properly. The power supply can supply up to 3 A, which allows for about an extra 1 A of wiggle room for the other components that may switch on for a short period of time.

### **Buck Converters**
The buck converter boards were found on Digi-key. The board uses the 	
LM2596 voltage regulator IC and, according to the datasheet provided by Digi-key for the board, it is suitable for our application.

Noteable specs include:
- Input voltage: 4 - 40 V
- Output voltage: 1.5 - 37 V
- Max output current: 3 A
- Max output power: 20 W
- Adjustable via screw potentiometer
- built in voltmeter

"A typical output ripple voltage can range from approximately 0.5% to 3% of the output voltage" - LM2596 datasheet

The output ripple at worst case will be $0.03(6) = 0.18 \ V = 180 \ mV$

Buck converter boards will be used to drop the 12 V supply voltage to 6 V for all components needing 6 V to operate. These buck converters are shown on the electrical schematic above.

Due to issues with LTSpice, further simulations will use voltage drops as a representation for the buck converter model. Using the buck converter model itself caused many issues in regards to the simulation. Spice would crash each time the buck converter model was added to a schematic, regardless of if it was connected or not.


### **Emergency stop button**

The emergency stop button will need to have a flyback diode connected to the two output lines of the button to prevent sudden voltage spikes when current is interrupted.

The E-stop button was chosen to be a 2 channel normally closed switch. This was chosen so power can be broken from locomotion, consumption and sorting all together with one button as shown in the electrical schematic above. 


### **Motor simulations**

#### **Spice Simulation of all motors**

![image](https://user-images.githubusercontent.com/112428796/217616689-fc20f8b4-5d28-49c7-8023-0d386eb82b25.png)

The resistor values were chosen through trial and error as a representation for the buck converter's voltage drop from 12 V to 6 V.


#### **Spice Simulation of all continuous motors**

![image](https://user-images.githubusercontent.com/112428796/217616760-0a5e9cba-0432-45b6-9b12-233098d659ca.png)

Above is the spice model for all continuously running motors. The $47 \ \mu F$ smoothing capacitors are connected across the terminals of each motor to protect against RF electromagnetic interference produced from the motor caused by the brushes causing current arcs. A flyback diode was also added to prevent a large voltage spike from damaging any components when the supply voltage is turned off.



## **BOM**
| Name of Item              | Description                                                     | Used in which subsystem(s) | Part Number   | Manufacturer     | Quantity | Price      | Total |
|---------------------------|-----------------------------------------------------------------|----------------------------|---------------|------------------|----------|------------|-------|
| Rechargeable 12 V Battery | 12 V/6000 mAH Lithium Ion Battery Pack                          | Power                      | YB1206000-USB | TalentCell       | 1        | 39.99      | 39.99 |
| Buck Converters           | DC-DC Adjustable Buck Converters 3-40V to 1.5-35V               | Power                      | LM2596        | ATNSINC          | 1        | 15.69      | 15.69 |
| E-Stop Button             | Self-Locking Emergency Stop Button 2 NC Red Mushroom 660 V 10 A | Power                      | HB2-BS544     | MXUTEUK          | 1        | 10.99      | 10.99 |
| 3 A Fuses                 | 3A 250V Fuses (pack of 20)                                      | Power                      | F3AL250V      | BOJACK           | 2        | 5.99       | 11.98 |
| 1N4001 Diode              | 50V, 1A Diode (100 pcs)                                         | Power                      | 1N4001        | MCIGICM          | 1        | 4.74       | 4.74  |
| 47 uF Capacitors          | 47uF 50V Electrolytic Capacitors (10 pack)                      | Power                      | 47 uF         | BOJACK           | 1        | 5.99       | 5.99  |
| Power Distribution Bus    | Solid Brass                                                     | Power                      | 737           | Adafruit         | 2        | 1.95       | 3.9   |
| Total                     |                                                                 |                            |               | Total Components | 9        | Total Cost | 93.28 |



