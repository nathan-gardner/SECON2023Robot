# Sorting Subsystem
## Function of The Subsystem 
- Take ducks and pedestals from the comsumption susbsytem to proper storage locations via a conveyor belt
- Actively sort pedestals while passively sorting ducks 


## Constraints
- The sorting system must be designed to be space efficient due to the robot having a size contraint of 1 cubic foot.
- The rate at which items come into the sorting subsystem
- The speed of the conveyor will constrain the speed of the flipper that will push pedestals
- The size of ducks will constrain the width of the conveyor belt. The belt must have a width wider than the duck's width of 3.5''.
- With a object freqency of  1 item every 5 seconds decided upon in the consumption subsystem, The flipper should be able to hit a pedestal and reset to resting position in half that time in order to ensure the flipper does not interfere with any other object.

## Buildable Schematic

there is nothing here rn lol hahah

## Analysis
### **Conveyor Belt**:

#### **Belt length**:
Conveyor length $L = 9in$ (Chosen)

$C$ is the 

center to center distance $C = 9 - 2(0.5) = 8\ in$

of the drive pulley $D$ (chosen to be 1 in)

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

$F =F_{fduck} + 3m_{duck}(g)(sin(\theta) +kcos(\theta)$

$F =0.7979 + 0.0708(9.8)(sin(0) +1.15cos(0)) =1.4917\ N$

$T_{L} = \frac{FD}{2\eta } =\frac{(1.4917)(0.0254)}{2(0.5)} = 0.03789 \ Nm$

$\ $
#### **Conveyor Speed:**
Chosen speed to be at least $2\  in/s = 10\ ft/min$

Speed of conveyor $s$

$s=D(RPM)(0.2618)(1.021)$

$\rightarrow  RPM = \frac{s}{(0.2618)(1.021)(D)} = \frac{10}{(0.2618)(1.021)(1)} = 37.5\  RPM$


$\ $
#### **Power Required: (may remove idk what all this mean)**
Approximate weight of rubber = $24.94\ g/cm^3 = 0.05498\ lbs/cm^3$

Belt Dimensions in inches = 19.1416 in X 3.75 in X 0.0625 in 

Belt Dimensions in cm = 48.57 cm X 9.525 cm X 0.15875 cm

Belt Volume $=(48.57)(9.525)(0.15875) = 73.44\ cm^3$

Weight of Belt $W_{belt} = (0.05498)(73.44) =4.0377 \ lbs $

Friction coefficient of rubber: $\ $ $k = 1.15$

$P = \frac{ks(W_{ducks}+W_{Belt})(745.7)}{33000}$

$P = \frac{(1.15)(5)(0.4683+4.0377)(745.7)}{33000} = 0.5855 \ W$

$\ $
#### **Conveyor Motor Requirements Summarized:**
$Torque > 0.03789 Nm$ or $3.86\ kgmm$

$RPM = 37.5$

The motor chosen meets all specifications.

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

time to hit pedestal and return to resting position:

$t \le 2.5 \ second $

$speed \ge 180\degree /2.5s$

The servo chosen meets all above requirements.



$\ $
### **Color Sensor:**
Electrical Specifications:

$V_{DD} = 3\ V$ 

$I_{DD} = 235\ \mu A \ \ (Active)$ 

$I_{DD} = 65\ \mu A \ \ (Wait)$ 

$I_{DD} = 2.5\ \mu A \ \ (Sleep)$ 

The above voltages and currents will be provided by the power subsystem

$\ $

Speed:

Clock Frequency: $\ \ 0-400kHz$

**INSERT IMAGE FROM DATASHEET FOR STATE MACHINE**

Detection will take a maximum of 616.4 ms.

Minimum flipper distance from sensor $=(0.6164\ ms)(2\ in/s) = 1.2328 \ in$

Flipper distance will be $1.5\ in$ for simplicity.

Wait about $250\ ms$ then activate flipper


$\ $

## BOMgit




