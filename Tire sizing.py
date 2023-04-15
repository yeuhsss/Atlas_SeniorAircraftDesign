import numpy as np
import math
import os

# Tire sizing 
W = 67549 # weight lbs
Na_b = .79768  
Mf_b = 0.15
Ma_b = 0.07
B = 37.07 #ft 
B_1 = 0.251
B_2 = 0.216
A_1 = 2.69
A_2 = 1.170
V_stall= 204.225
g = 32.17

# static load 

MSL_Main = W * Na_b

Max_SL_Nose = W * Mf_b

Min_SL_Nose = W * Ma_b 


print(MSL_Main)
print(Max_SL_Nose)
print(Min_SL_Nose)

#Kinetic Brake engery 
KBE = (1/2) * W/g * V_stall**2

print(KBE)

# Tire Size 
Diameter = A_1 * W**B_1

Width =  A_2 * W**B_2 

print(Diameter)
print (Width)