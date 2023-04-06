# **Sorting Subsystem** - Not Implemented

Due to shipping issues, this subsystem was cut.

## **Function of The Subsystem** 
- Take ducks and pedestals from the comsumption susbsytem to their proper storage locations via a conveyor belt and flipper.
- Actively sort pedestals while passively sorting ducks. 
- Objects will be passed from the consumption subsystem to the sorting subsystem. The sorting subsystem will then send the ducks to the duck storage and delivery subsystem while sending the pedestals to the pedestal storage and delivery subsystem.
- The motor driver for the DC motor will be controlled via the low-level controller subsystem.
- The driver for the servo motor is a part of the duck storage and delivery subsystem which will be controlled by the low-level controller subsystem.


## **Constraints**
### **Size:**
- The sorting subsystem must be designed to be space efficient due to the robot having a size constraint of 1 cubic foot. The consumption subsystem takes up $5 \ in$ of depth, so the sorting subsystem must be no longer than $7 \ in$ deep. The sorting subsystem must be at the top of the robot due to the consumption susbsytem feeding objects out to the top of the robot. There must be sufficient space for a pedestal silo and funnel under the sorting subsystem. The silo will be $5.5 \ in$ tall. The funnel has been designed to be $2.75 \ in$ tall, thus making the funnel and silo together $8.25 \ in$ tall. The conveyor will be above the funnel with a height of $1 \ in$. The flipper will be no greater than $1.5 \ in$ tall. This makes the total height of the sorting subsystem $10.75 \ in$. This number was found by adding the height of the funnel and silo with the height of the conveyor and maximum height of the flipper.
### **Conveyor:**
- The size of ducks will constrain the width of the conveyor belt. The belt must have a width wider than the duck's width of 3.5 $\ in$.
- The conveyor belt must effectively move ducks and pedestals at $2\ in/s$ and support the weight of at least three ducks ( $F_{ducks} = 2.08152\  N$ ).
- The length of the conveyor must be no more than $7 \ in$ to make room for the consumption subsystem (the consumption subsystem is $5\ in$ deep into the robot).
### **Flipper:**
- Flipper must be long enough to reach across the whole conveyor belt width ( $3.5 \ in$ )
- The flipper must be tall enough to push a pedestal and not cause spin or disturb other objects. The chosen width is $1.5 \ in$.
- The servo and flipper must provide enough force to move the pedestals ( $F_{flipper} \gt 1.5572 \ N$ ).
### **Sensor:**
- Color sensor and flipper must be at least $1.238\  in$ from each other to ensure the sensor has adequate time to detect the color of the object that passes by. More distance may be required to accomodate the speed of the microcontroller that will control the servo for the flipper. See the color sensor section under the analysis section for more details.
- The color sensor must be able to distinguish between pink, yellow, red, green, and white.
- Color sensor must work well with the Arduino architecture.
### **Socioeconomic:**
- Some parts were chosen to be 3D-printed instead of purchased in order to save cost.

## **Buildable Schematic**

### **CAD Model for the subsystem:**

![image](https://user-images.githubusercontent.com/30758520/217083573-140bcb9c-4fa0-4183-8102-0b3e01a4543d.png)

![image](https://user-images.githubusercontent.com/30758520/217083593-f0c92dea-f0be-456a-b0dd-45a2c0516af1.png)

![image](https://user-images.githubusercontent.com/30758520/217083626-1b335fac-fea3-4333-bb96-94edd8fac186.png)

![image](https://user-images.githubusercontent.com/30758520/217083646-45ddf2e9-122f-4edd-b5d9-c58c59687672.png)

![image](https://user-images.githubusercontent.com/30758520/217083665-4cb1557c-2bf3-4bbf-a9e0-518da7e4f294.png)

![image](https://user-images.githubusercontent.com/30758520/217083683-8d7532cf-3541-49e0-afe2-7faec3951319.png)

![image](https://user-images.githubusercontent.com/30758520/217083729-15961e79-865e-4e81-8c71-2f9a680bb755.png)

![image](https://user-images.githubusercontent.com/30758520/217083747-a606dd09-1a91-487c-baf5-59849e43aa57.png)

![image](https://user-images.githubusercontent.com/112428353/217404567-798dbee2-e0ad-4e2a-99f7-5935e4b5d46f.png)


### **Electrical Schematics:**

![image](https://user-images.githubusercontent.com/112428796/218278721-7a10bdb2-1479-46bc-87bb-2c08a4d699c8.png)

## **Analysis**
### **Conveyor Belt**:

#### **Belt length**:
Conveyor length $L = 7in$ (Chosen)

$C$ is the center to center distance $C = 7 - 2(0.5) = 6\ in$ of the drive pulley $D$ (chosen to be 1 in)

Belt Length ( $l$ )

$l = D\pi + 2C = 1\pi +2(6) = 15.1416\ in$

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

$T_{L} = \frac{FD}{2\eta } =\frac{(1.4917)(0.0254)}{2(0.5)} = 0.03789 \ N \ast m$

$\ $
#### **Conveyor Speed:**
Chosen speed to be at least $2\  in/s = 10\ ft/min$

Speed of conveyor $s$

$s = D(RPM)(0.2618)(1.021)$

$\rightarrow  RPM = \frac{s}{(0.2618)(1.021)(D)} = \frac{10}{(0.2618)(1.021)(1)} = 37.5\  RPM$


$\ $
#### **Power Required:**

Approximate weight of rubber = $24.94\ g/cm^3 = 0.05498\ lbs/cm^3$

Belt Dimensions in inches = 15.1416 in X 3.75 in X 0.0625 in 

Belt Dimensions in cm = 38.46 cm X 9.525 cm X 0.15875 cm

Belt Volume $=(38.46)(9.525)(0.15875) = 58.155\ cm^3$

Weight of Belt $W_{belt} = (0.05498)(58.155) = 3.1974 \ lbs $

Friction coefficient of rubber: $k = 1.15$

$P = \frac{ks(W_{ducks}+W_{Belt})(745.7)}{33000}$

$P = \frac{(1.15)(5)(0.4683+3.1974)(745.7)}{33000} = 0.5855 \ W$

$\ $
#### **Conveyor Motor Requirements Summarized:**
$Torque > 0.03789 N \ast m$ or $3.86\ kg \ast mm$

$RPM = 37.5$

Below is the torque curve for the motor chosen from Pololu.

![image](https://user-images.githubusercontent.com/30758520/215600423-c72d078f-4e15-4f09-ad2c-ea4ddf9deed2.png)

Therefore, the motor chosen meets all specifications.

According to the datasheet for the motor driver, it will be able to supply the necessary voltage and current values (6 V, 250 mA). This is the same motor driver used for the consumption motor which requires double the current compared to this subsystem

$\ $
### **Flipper:**
The flipper consists of a servo motor connected to a 3D printed plastic "pinball" flipper.


weight of ABS $1.03 \frac{g}{cm^3}$

Assuming the plastic flipper is rectangular:

 $3.5in$ x $1in$ x $1.5in$ = $5.25 \ in^3$ = $86.03209 \ cm^3$

 $W_{block} = (1.03 \frac{g}{cm^3})(86.03209 \ cm^3) = 88.6130527 \ g = 0.0886 \ kg$

$(0.0886 \ kg)(9.81) = 0.869 N$

Force needed to push block attached to the servo:

$F_{fplastic}= 0.869 \ N$


Force needed to push the pedestal on rubber:

$F_{fped}= 0.2322 \ N$

Total force needed to be applied by the servo:

$F_{flipper}= 0.2322 \ N + 1.325 \ N = 1.5572 \ N$

The flipper must apply more force than this in order to push the pedestal into the funnel.

$\tau = F_{flipper}(r)(sin(\theta)) = 1.5572(0.0889)(sin(90)) = 0.13 \ N \ast m$







$\ $
### **Color Sensor:**

Electrical Specifications:

$V_{DD} = 3\ V$ 

$I_{DD} = 235\ \mu A \ \ (Active)$ 

$I_{DD} = 65\ \mu A \ \ (Wait)$ 

$I_{DD} = 2.5\ \mu A \ \ (Sleep)$ 

The above voltages and currents will be provided by one of the Arduino Megas from the low-level controller subsystem.

$\ $

Speed:

Clock Frequency: $\ \ 0-400kHz$

![image](https://user-images.githubusercontent.com/112428796/203214738-1178d2db-62f4-489b-8cfd-b6a167bece1f.png)

Above is the state machine representation for the sensor circuit showing the times each of the states will take. For the majority of the time, the sensor will be in the states idle, RGCB ADC and RGCB INIT after the startup. Detection will take a maximum of 616.4 ms.

Pedestals have a diameter of $2 \ in$ and the conveyor moves at $2 \ \frac{in}{s}$ which means that one point of the pedestal will be in front of the sensor for 1 second which allows for plenty of time for the sensor to detect the color of the object as detection takes a maximum of 0.614 ms.

Minimum flipper distance from sensor $=(0.6164\ s)(2\ in/s) = 1.2328 \ in$

Flipper distance will be $1.5\ in$ for simplicity.



$\ $

## **BOM**

| Name of Item    | Description                                                                                                      | Used in which subsystem(s) | Part Number | Manufacturer     | Quantity | Price      | Total  |
| --------------- | ---------------------------------------------------------------------------------------------------------------- | -------------------------- | ----------- | ---------------- | -------- | ---------- | ------ |
| Color Sensor        | RGB Color Snesor with IR filter and White LED                                                                    | Sorting | TCS34725 | Adafruit         | 1  | $7.95      | 7.95   |
| Servo Motor         | Analog Servo                                                                                          | Sorting | 2818    | Fitech           | 1  |  $5.25          |  $5.25      |
| Motor               | 99:1 Metal Gearmotor 25Dx69L mm LP 6V with 48 CPR Encoder                                                        | Sorting | 4827     | Pololu           | 1  | $45.95     | 45.95  |
| Funnel              | 3d printed                                                                                                       | Sorting | N/A      | N/A              | 1  |            | 0      |
| Flipper             | 3d printed                                                                                                       | Sorting | N/A      | N/A              | 1  |            | 0      |
| Rubber Belt         | Rubber-Cal Heavy Black Conveyor Belt - Rubber Sheet - .30(2Ply) Thick x 10"" Width x 4"" Length - Black (3 Pack) | Sorting | N/A      | Rubber-cal       | 1  | $56        | 55.65  |
| Rollers             | 3d printed                                                                                                       | Sorting | N/A      | N/A              | 2  |            | 0      |
| DC Motor Controller | TB9051FTG Single Brushed DC Motor Driver Carrier                                                                 | Sorting | 2997     | Pololu           | 1  | $11.95     | 11.95  |
| Motor Mount         | Pololu 25D mm Metal Gearmotor Bracket Pair                                                                       | Sorting | 2676     | Pololu           | 1  | $7.95      | 7.95   |
| Total               |                                                                                                                  |         |          | Total Components | 10 | Total Cost | 134.70 |
