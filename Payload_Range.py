#### EAE130B - Spring 2023
### Payload Range Estimation Tool 
##Author Zak Staub

import numpy as np
import matplotlib.pyplot as plt


#This tool is used to estimate the payload range of our aircraft for both conventional 
# and Hybrid electric applications. 
#  Enter parameters for the aircraft below and this tool will output a 
# chart displaying the range of the aircraft



#Variable Initialization 

# Assumed at cruise for max range

V =  590.8 #Velocity in [ft/s]
AR =  10.061 # Aspect Ratio
e = 0.8 # Oswald Efficiency Factor 
rho = 0.0009585182 # Density at 28000 ft (cruise) [slugs/ft^3]
S = 805.06 # Wing Area [ft^2]
b = 90 # Wing Span[ft]
C_Do = 0.022 # Coeff of Drag 
C_L = 1.6 # Coeff of lift at L/D max
N_p = .9 # propeller efficiency 
c = 9.5 # Chord length [ft]
Fuel_Max = 12900 # Max Weight of Fuel (includes 6% reserve) [lbf]
Fuel_Min= 6450 # Min fuel needed to meet 500 nmi (includes 6% reserve) [lbf]
Payload_Max = 12660 # Maximum Payload [lbf]
OEW = 48439 # Operational Empty Weight [lbf]
MTOW = 67549 # Maximum Takeoff Weight [lbf]



# Conversion Factor 
slug = 32.17 # [lbm] in a slug
nmi = 6076.12 # [ft] in a nmi 

# Fully Fuel Estimation 

# Calculations 

# For Conventional estimation, airspeed, altitude, c, and prop efficiency 
# are assumed constant. Lift = Weight 

Res_Fuel_Min = (Fuel_Min / 1.06) * .06 # Calcs reserve fuel [lbf]
Res_Fuel_Max = (Fuel_Max / 1.06) * .06 # Calcs reserve fuel [lbf]

# Variable to be used in the Range Equation 
 
a1a2 = np.sqrt(C_Do / (np.pi * AR * e))

a2_by_a1 = (1 / (.5 * rho * (V ** 2) * S * C_L))

#Starting Weights Point B 
Wo_B = MTOW # Max takeoff Weight [lbf]
W1_B = OEW + Payload_Max + Res_Fuel_Min# OEW (operating empty weight) + Max payload + Reserve Fuel [lbf]

#Starting Weights Point C
Wo_C = MTOW # Max takeoff Weight [lbf]
W1_C = MTOW - Fuel_Max + Res_Fuel_Min # Max takeoff Weight - Max Fuel + Reserve Fuel at Payload =(MTOW - Max Fuel - OEW)[lbf]

#Starting Weigths Point D 
Wo_D = Fuel_Max + OEW  # Max Fuel + OEW  [lbf]
W1_D = OEW + Res_Fuel_Min # OEW + Reserve Fuel [lbf]

# Payloads 

P_loadA = Payload_Max # Max Payload (total) [lbf] at point A 
P_loadB = P_loadA # Payload [lbf] at point B not at max fuel
P_loadC = MTOW - Fuel_Max - OEW  # Reduced Payload [lbf] for max fuel at point C 
P_loadD = 0 # No Payload [lbf] at point D for max range


def RangeCalc(Wo,W1):
    
    R = (N_p / c) * (1 / a1a2) * (np.arctan(Wo * (a2_by_a1)) - np.arctan(W1 * (a2_by_a1)))
    
    R = R * nmi # conversiion from [ft] to [nmi]
    
    return R

R_A = 0 # Starting Point 

R_B = RangeCalc(Wo_B, W1_B) # Range at point B 

R_C = RangeCalc(Wo_C, W1_C) # Range at point C 

R_D = RangeCalc(Wo_D, W1_D) # Range at point D

# Plotting Range 

plt.figure(figsize = (8,8))
plt.title("Payload vs Range Estimation", size = 20)
Point1 = [R_A ,P_loadA]
Point2 = [R_B ,P_loadB]
Point3 = [R_C ,P_loadC]
Point4 = [R_D ,P_loadD]
x_values = [Point1[0], Point2[0],Point3[0],Point4[0]]
y_values = [Point1[1], Point2[1],Point3[1], Point4[1]]
plt.plot(x_values, y_values, 'black', linestyle="-")

plt.plot(R_A, P_loadA, marker="o",  color = "green", label = "Point A")
plt.plot(R_B, P_loadB,  marker="o",  color = "blue" , label = "Point B")
plt.plot(R_C, P_loadC,  marker="o", color = "orange", label = "Point C")
plt.plot(R_D, P_loadD,  marker="o", color = "red", label = "Point D")
plt.ylabel("Payload [lbf] ",  size =15)
plt.xlabel("Range [nmi]", size= 15)
plt.legend()

