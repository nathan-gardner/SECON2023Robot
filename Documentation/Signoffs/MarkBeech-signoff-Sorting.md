# Sorting Subsystem
## Function of The Subsystem 
- Take ducks and pedestals from the comsumption susbsytem to their proper storage locations via a conveyor belt and flipper
- Actively sort pedestals while passively sorting ducks 


## Constraints
- The sorting system must be designed to be space efficient due to the robot having a size constraint of 1 cubic foot.
- The size of ducks will constrain the width of the conveyor belt. The belt must have a width wider than the duck's width of 3.5 $\ in$.
- With a object freqency of 1 item every 5 seconds decided upon in the consumption subsystem, The flipper should be able to hit a pedestal and reset to resting position in half that time in order to ensure the flipper does not interfere with any other object.

## Buildable Schematic

### **CAD Model for the subsystem:**

![image](https://user-images.githubusercontent.com/112428796/203214411-7822e98e-555d-492a-9ce3-bfae94a87829.png)

![image](https://user-images.githubusercontent.com/112428796/203214451-758f40d0-eb9d-43b1-9ca9-24e6db49db30.png)

![image](https://user-images.githubusercontent.com/112428796/203214470-0e758ce9-b856-4eec-923e-32410e9d9563.png)

![image](https://user-images.githubusercontent.com/112428796/203214491-ff77c0a5-24d4-42c6-9161-67491ab186df.png)

![image](https://user-images.githubusercontent.com/112428796/203214515-7af0d62c-5a34-484e-a266-4728f000bd4a.png)

### **Electrical Schematics:**

![image](https://user-images.githubusercontent.com/112428796/203215313-85ff081d-fb19-4ab0-b8c6-72a5adc10ec5.png)

Above is the circuit schematic for the color sensor PCB chosen from Adafruit

![image](https://user-images.githubusercontent.com/112428796/203216260-575a3c3f-c26e-4205-bc9b-af41570a12e8.png)

Above shows the connection to the Arduino MEGA from the controller subsystem. The digital pins and PWM pins will be used to control the carious electrical components for the system. The PWM pin will be used to control the motor for the conveyor belt. The digital pins will be used for the servo and the the color sensor.

## Analysis
### **Conveyor Belt**:

#### **Belt length**:
Conveyor length $L = 9in$ (Chosen)

$C$ is the center to center distance $C = 9 - 2(0.5) = 8\ in$ of the drive pulley $D$ (chosen to be 1 in)

Belt Length ($l$)

$l = D\pi + 2C = 1\pi +2(8) = 19.1416\ in$

$\ $
#### **Normal and Frictional Forces**:
Worst case scenario assumed to be 3 ducks on the conveyor at once:

Force Calculations:

Normal Force of one duck:

$F_{Nduck} =(0.0708\ kg)(9.8\ m/s) = 0.6938\  N$

Force of three ducks on the conveyor belt:

 $F_{ducks} =3(0.0708\ kg)(9.8\ m/s) = 2.08152\  N$

Weight of three Ducks:

$W_{ducks} = (0.225)(2.08152) = 0.4683\ lbs$

Friction Force of Duck:

$F_{fduck} = kF_{Nduck} = (1.15)(0.0708\ kg)(9.8\ m/s) = 0.7979 \ N$

Normal Force of one pedestal:

$F_{Nped}=(0.0206\ kg)(9.8\ m/s) = 0.20188$


Friction Force of Pedestal:

$F_{fped} = kF_{Nped} = (1.15)0.20188= 0.2322 \ N$

$\ $


### **Torque needed**:

Efficiency: $\eta$ Assumed to be 50%

Assuming worst case of three ducks:

$F = F_{fduck} + 3m_{duck}(g)(sin(\theta) + kcos(\theta)$

$F = 0.7979 + 0.0708(9.8)(sin(0) + 1.15cos(0)) = 1.4917\ N$

$T_{L} = \frac{FD}{2\eta } =\frac{(1.4917)(0.0254)}{2(0.5)} = 0.03789 \ Nm$

$\ $
#### **Conveyor Speed:**
Chosen speed to be at least $2\  in/s = 10\ ft/min$

Speed of conveyor $s$

$s = D(RPM)(0.2618)(1.021)$

$\rightarrow  RPM = \frac{s}{(0.2618)(1.021)(D)} = \frac{10}{(0.2618)(1.021)(1)} = 37.5\  RPM$


$\ $
#### **Power Required: **

Approximate weight of rubber = $24.94\ g/cm^3 = 0.05498\ lbs/cm^3$

Belt Dimensions in inches = 19.1416 in X 3.75 in X 0.0625 in 

Belt Dimensions in cm = 48.57 cm X 9.525 cm X 0.15875 cm

Belt Volume $=(48.57)(9.525)(0.15875) = 73.44\ cm^3$

Weight of Belt $W_{belt} = (0.05498)(73.44) =4.0377 \ lbs $

Friction coefficient of rubber: $k = 1.15$

$P = \frac{ks(W_{ducks}+W_{Belt})(745.7)}{33000}$

$P = \frac{(1.15)(5)(0.4683+4.0377)(745.7)}{33000} = 0.5855 \ W$

$\ $
#### **Conveyor Motor Requirements Summarized:**
$Torque > 0.03789 Nm$ or $3.86\ kgmm$

$RPM = 37.5$

Therefore, the motor chosen meets all specifications.

$\ $
### **Flipper:**

Force needed to push the pedestal on rubber:

$F_{fped}= 0.2322 \ N$

The flipper must apply more force than this in order to push the pedestal into the funnel.

$r = 2\ in =0.0508 \ m$

$\tau = rFsin(\theta)$

$\tau_{max} = (0.0508)(0.2322)(sin(\pi)) = 0.0118 \ Nm = 1.202\  kgmm$



$f_{items} = \frac{1 \ item}{5 \ second}$

$s_{belt}=\frac{2\ in}{second}$

Time required to hit pedestal and return to resting position:

$t \le 2.5 \ second $

$speed \ge 180\degree /2.5s$

Therefore, the servo chosen meets all above requirements.



$\ $
### **Color Sensor:**

![image](https://user-images.githubusercontent.com/112428796/203215313-85ff081d-fb19-4ab0-b8c6-72a5adc10ec5.png)

Above is the circuit schematic for the color sensor PCB chosen from Adafruit


Electrical Specifications:

$V_{DD} = 3\ V$ 

$I_{DD} = 235\ \mu A \ \ (Active)$ 

$I_{DD} = 65\ \mu A \ \ (Wait)$ 

$I_{DD} = 2.5\ \mu A \ \ (Sleep)$ 

Therefore, the above voltages and currents will be provided by the power subsystem

$\ $

Speed:

Clock Frequency: $\ \ 0-400kHz$

![image](https://user-images.githubusercontent.com/112428796/203214738-1178d2db-62f4-489b-8cfd-b6a167bece1f.png)

Above is the state machine representation for the sensor circuit showing the times each of the states will take. For the majority of the time, the sensor will be in the states idle, RGCB ADC and RGCB INIT after the startup. Detection will take a maximum of 616.4 ms.

Minimum flipper distance from sensor $=(0.6164\ ms)(2\ in/s) = 1.2328 \ in$

Flipper distance will be $1.5\ in$ for simplicity.

Wait about $250\ ms$ then activate flipper



$\ $

## BOM
| Name of Item | Description                                                                                                            | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total |
|--------------|------------------------------------------------------------------------------------------------------------------------|----------------------------|-------------|------------------|----------|------------|-------|
| Color Sensor | RGB Color Snesor with IR filter and White LED                                                                          | Sorting                    | TCS34725    | Adafruit         | 1        | $7.95      | 7.95  |
| Servo Motor  | closed loop servo motor                                                                                                | Sorting                    | HS-40       | Hitec            | 1        | $10.72     | 10.72 |
| Motor        | 99:1 Metal Gearmotor 25Dx69L mm LP 6V with 48 CPR Encoder                                                              | Sorting                    | 4827        | Pololu           | 1        | $45.95     | 45.95 |
| Funnel       | 3d printed                                                                                                             | Sorting                    | N/A         | N/A              | 1        |            | 0     |
| Flipper      | 3d printed                                                                                                             | Sorting                    | N/A         | N/A              | 1        |            | 0     |
| Rubber Belt  | "Rubber-Cal Heavy Black Conveyor Belt - Rubber Sheet - .30(2Ply) Thick x 10"""" Width x 4"""" Length - Black (3 Pack)" | Sorting                    | N/A         | Rubber-cal       | 1        | $56        |       |
| Rollers      | 3d printed                                                                                                             | Sorting                    | N/A         | N/A              | 2        |            |       |
| Total        |                                                                                                                        |                            |             | Total Components | 8        | Total Cost | 64.62 |




