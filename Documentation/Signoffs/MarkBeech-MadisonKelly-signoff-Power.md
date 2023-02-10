# **Power Subsystem**
## **Function of The Subsystem** 
- Provides power to all other subsystems that require voltages and currents not supplied indirectly through the low-level controllers or the top level controller pins.
- Allows the robot to be turned off and on.
- Addition of an emergency stop button in order to meet safety standard and competition requirement.
## **Constraints** 
- The power supply must supply 12 V and a current of more than 2 A to accommodate all components that will be running at one time. 
- The output of the battery must be regulated with a boost converter in order to ensure each component is provided with the proper voltage for operation.   
- Fuses will need to be added to the power supply bus for added overcurrent protection.
- Many of the loads will need to be stepped down to 6V and have a regulated output to eliminate ripple voltage. There will be filters to smooth out the ripple
- The ripple voltage from the inductive load of the motors should be eliminated in order to protect other components. To mitigate this, the team will add smoothing capacitors on each motor load.
- Maximum allowable ripple voltage on all components will be less than $50 \ mV_{p-p}$ to prevent damage to components.
- Since there are multiple loads that need 12V and 6V supplied, there will need to be a power bus for each of them. The power bus selected is rated up to 10A.
- The power supply chosen has a 5V USB output. Because of this, the concerns for powering the Nvidia Jetson as well as both arduinos will be separate from the remainder of the components. 
- An emergency stop button must be used for the safety of those involved in the competition. 
  - The emergency button must have a flyback diode attached in reverse bias because of the motor’s inductive load and the chance of a sudden switch in the power being supplied.

### **Constraint from Standard**

- Shall have a self-latching emergency stop push-button that has a positive operation. The button shall not be a graphical representation or a flat switch based on NFPA 79 - 10.7.2.


### **Voltage and current requirements for all components**

| Subsystem                                  | Component                  | Voltage (V) | Current (A) |
| ------------------------------------------ | -------------------------- | ----------- | ----------- |
| Locomotion                                 | Motor Drivers (2)          | 12          | 1.2         |
| Consumption                                | Motor Driver               | 6           | 0.5         |
| Duck Storage and Delivery                  | Solenoid Actuator          | 12          | 0.65        |
| Pedestal Storage, Duck Storage, Sorting, and Feeding | Servo Motor Controller | 6     | 2.17        |
| Sorting                                    | Motor Controller           | 6           | 0.25        |


### **Voltage and current requirements for all continuous components**

| Subsystem  | Component | Voltage (V) | Current (A)  |
| ----------- | -------------------------- | -- | ---- |
| Locomotion  | Motor Drivers - Motors (4) | 12 | 1.2  |
| Consumption | Motor Driver - Motor       | 6  | 0.5  |
| Sorting     | Motor Controller           | 6  | 0.25 |

## **Buildable Schematic**

### **CAD Model for the subsystem:**

Shown below is the layout of the robot. The compact size of this power supply (shown in red) should allow for plenty of room for other components as needed.

![image](https://user-images.githubusercontent.com/112428353/217404755-75b5127b-590a-4b0a-8966-9f2886d97de1.png)


### **Electrical Schematics:**

All electrical circuits will be implemented via soldered breadboard.

![image](https://user-images.githubusercontent.com/112424739/217995287-52ce555a-2eab-46a1-9221-4d2651baa916.png)

## **Analysis**

### **Power Supply** 

The power supply can supply 12 V at 6000 mAH and 5 V at 12000 mAH and the maximum current output is 3A. Since each competition round is only 3 minutes, the following calculations were performed. 

$Time = \frac{6000\ mAh}{3\ A} = 2\ hours = 120\ minutes$

$rounds = \frac{120}{3} = 40 rounds$

The robot should be able to run up to 40 rounds without needing a charge. This will help with testing as well as reducing the risk that the robot's power supply will die during the competition.

The power supply's 5 V, 2 A output will be connected via a USB A to USB A cable to the Nvidia Jetson Nano. 


### **Current Analysis**

#### **Block Diagram for all components**

![image](https://user-images.githubusercontent.com/112428796/217997468-f6a13fab-15ee-4600-b607-916deef9cabb.png)

Currents have been labelled for use in a matlab script detailed below. Each current value listed by each component was found from the components datasheet.

#### **Block diagram for all constant loads**

![image](https://user-images.githubusercontent.com/112428796/217997561-0201fd06-febe-4eff-a30a-7c7635510d9f.png)

#### **Matlab Simulation**

![image](https://user-images.githubusercontent.com/112428796/218004508-6a2b09b6-df3f-4aab-b1d5-3a6f0b38a0dd.png)

This simulation was done using the block diagram for all constant loads shown above.

The calculates the currents going into each branch and adds them together to find the total current needed to be supplied by the power supply. This was done while taking into account the efficiencies of the buck converter and the boost converter.

According to the MatLab simulation above the total current needed to be supplied by the power supply is 1.7862 A. The power supply chosen has a max output current of 3 A, which is almost double the constant current constraint.

### **Boost Converters**

In order to ensure the proper voltage for each component, the team will use a boost converter on the output of the power supply. Since the power supply has an unregulated voltage output of 12.6-9V, the boost converter will take the output voltage of the battery and step the voltage up to 12V when needed. The concern for noise produced by the buck converter is adressed in an LTSpice simulation shown below.

![image](https://user-images.githubusercontent.com/112428796/218002431-433ad5b6-c22e-4e8f-a45d-0492a3a7a638.png)

According to the datasheet for the boost converter chosen, the ripple noise has a peak to peak ampliture of 220 mV and a switching frequency of 220 kHz. The noise has been modelled as such and can be seen in the LTSpice model above.

![image](https://user-images.githubusercontent.com/112428796/218002477-c4cee8fd-4225-4f78-bced-337f54ca0a94.png)

Above is the unregulated noise before passing through the filter. The filter will be implemented via a solderless breadboard.

![image](https://user-images.githubusercontent.com/112428796/217997738-3bb06fd8-8bd1-408b-a109-000978781226.png)

Above is the output voltage after the filter. The voltage appears to be much claner and varries between 12.0003 V and 11.0097 V which is a variation of less than 1% from 12 V.

### **Buck Converters**
The buck converter boards were found on Digi-key. The board uses the 	
LM2596 voltage regulator IC and, according to the datasheet provided by Digi-key for the board, it is suitable for our application.

Noteable specs include:
- Input voltage: 4 - 40 V
- Output voltage: 1.5 - 37 V
- Max output current: 3 A
- Max output power: 20 W
- Switching frequency: 150 kHz
- Adjustable via screw potentiometer
- built in voltmeter

Buck converter boards will be used to drop the 12 V supply voltage to 6 V for all components needing 6 V to operate. These buck converters are shown on the electrical schematic above. 

According to the datasheet for the LM2596 part, a typical output ripple voltage can range from approximately 0.5% to 3% of the output voltage. This means The output ripple at worst case will be:

 $0.03(6) = 0.18 \ V = 180 \ mV$


![image](https://user-images.githubusercontent.com/112428796/218002527-55c4fd77-e1b0-4a0a-b55e-a9d1a5be6ea4.png)

Above is an LTSpice simulation of the noise ripple that could result from the buck converter board. The device has a switching frequency of 150 kHz as reflected in the model.

![image](https://user-images.githubusercontent.com/112428796/218002596-143b8e32-88fc-4f3e-ad19-f684000c760d.png)

Above is the unregulated noise signal with $V_{p-p} = 180 mV$ and a frequency of 150 kHz. A LC filter with a diode will be implemented on a solderless breadboard to flatten out the voltage.

![image](https://user-images.githubusercontent.com/112428796/217917663-eb27452f-6659-4c17-b1c0-8ecb8b31a864.png)


As shown above the output voltage is regulated to 6 V with a small ripple of less than 1% of the output voltage.

### **Motor Ripple**

All motors chosen in this project have come from Pololu. Pololu recommends connecting up to three $0.1 \mu F$ capacitors to the motors. It was recommended that one cap should be connected from the positive lead to the case, one from the negative lead to the case, and one across the input terminals.

### **Power Bus**

Since there are many components that need 12V and 6V, there will be at least two power buses to better distribute the power to the correct components within the robot. The buses selected are rated up to 24V, and 12V is the maximum voltage within the robot. Also, the number of outputs on the bus itself is much greater than the number of outputs the team will need. Therefore, the buses selected will be sufficient. 

### **Nvidia Jetson Power**

Since the power supply chosen has a separate 5V USB port output with separate ratings, the analysis for the Nvidia Jetson is simple. The team will use the USB(A)-USB(A) that came with the Nvidia Jetson to supply the voltage needed. 

### **Emergency stop button**

The emergency stop button will need to have a flyback diode connected to the two output lines of the button to prevent sudden voltage spikes when current is interrupted.

The E-stop button was chosen to be a 2 channel normally closed switch. This was chosen so power can be broken from locomotion, consumption and sorting all together with one button as shown in the electrical schematic above. 



## **BOM**
| Name of Item              | Description                                                                                                                                          | Used in which subsystem(s) | Part Number   | Manufacturer     | Quantity | Price      | Total  |
|---------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------|---------------|------------------|----------|------------|--------|
| Rechargeable 12 V Battery | 12 V/6000 mAH Lithium Ion Battery Pack                                                                                                               | Power                      | YB1206000-USB | TalentCell       | 1        | 39.99      | 39.99  |
| Buck Converters           | DC-DC Adjustable Buck Converters 3-40V to 1.5-35V                                                                                                    | Power                      | LM2596        | ATNSINC          | 1        | 15.69      | 15.69  |
| E-Stop Button             | Self-Locking Emergency Stop Button 2 NC Red Mushroom 660 V 10 A                                                                                      | Power                      | HB2-BS544     | MXUTEUK          | 1        | 10.99      | 10.99  |
| 3 A Fuses                 | 3A 250V Fuses (pack of 20)                                                                                                                           | Power                      | F3AL250V      | BOJACK           | 2        | 5.99       | 11.98  |
| 1N5822 Diode              | Ximimark 100Pcs 1N5822 Schottky Diode 3A 40V DO-201AD (DO-27) Barrier Rectifier Diode for Household Appliances                                       | Power                      | 1N5822        | Ximimark         | 1        | 7.29       | 7.29   |
| 0.1 uF Capacitors         | E-Projects - 0.1uF Ceramic Disc Capacitor - 50 Volts (25 Pieces)                                                                                     | Power                      | 0.1 uF        | E-Projects       | 1        | 5.99       | 5.99   |
| 330 uF Capacitors         | Capacitor 4.7UF 6.8UF 10UF 15UF 22UF 47UF 100UF 220UF 330UF 470UF 680UF 1500UF Electrolytic Capacitors Kit 6.3V 10V 16V 25V 35V 50V 100V 400V,295Pcs | Power                      | 330 uF        | changhe          | 1        | 18.98      | 18.98  |
| 33 uH inductors           | uxcell 50Pcs 0510 Color Ring Inductor 33uH 1W Axial RF Choke Coil Inductor                                                                           | Power                      | 33 uH         | uxcell           | 1        | 8.49       | 8.49   |
| Boost Converter           | DC DC Converter 12 V 120 W                                                                                                                           | Power                      | ?             | ?                | 1        | 0          | 0      |
| Power Distribution Bus    | Solid Brass                                                                                                                                          | Power                      | 737           | Adafruit         | 5        | 1.95       | 9.75   |
| Total                     |                                                                                                                                                      |                            |               | Total Components | 15       | Total Cost | 129.15 |




