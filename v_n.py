import numpy as np
import matplotlib.pyplot as plt

def v_n(W,nmax,nmin,rho,CL,S,Vc_ktas,cbar,CLalf,title1,title2): # Adjust input variables as necessary

    # EAS Velocities
    Vc = (Vc_ktas*1.688)*((rho/0.00237)**(0.5))
    Vd = 1.25*Vc
    Vs = 121
    W_S = W/S
    mu = (2*W_S)/(rho*cbar*CLalf*32.2)
    kg = (0.88*mu)/(5.3+mu)
    ude_c = 56
    ude_d = ude_c*0.5
    ude_b = ude_c
    Vc_eas_kn = Vc/1.688
    Vb = Vs*(1+(kg*ude_c*Vc_eas_kn*CLalf)/(498*W_S))
    Vb_eas_kn = Vb/1.688
    Vd_eas_kn = Vd/1.688

    # Flight loads
    V = np.linspace(0,Vd,10000)
    n_Stall = (0.5*rho*CL*(V**2))/W_S
    n_pos = np.ones(10000)*nmax
    n_neg = -nmin*np.ones(10000)
    upper_idx = np.argwhere(np.diff(np.sign(n_Stall - n_pos))).flatten()
    lower_idx = np.argwhere(np.diff(np.sign((-1*n_Stall) - n_neg))).flatten()

    # Gust Loads
    V_rough_array = np.linspace(0,Vb,10000)
    V_rough_array_kn = np.linspace(0,Vc_eas_kn,10000)
    V_cruise_array = np.linspace(0,Vc,10000)
    V_cruise_array_kn = np.linspace(0,Vc_eas_kn,10000)
    V_dive_array = V
    V_dive_array_kn = np.linspace(0,Vd_eas_kn,10000)
    n_rough_pos = 1+(kg*CLalf*ude_b*V_rough_array_kn)/(498*W_S)
    n_rough_neg = 1+(-1*kg*CLalf*ude_b*V_rough_array_kn)/(498*W_S)
    n_cruise_pos = 1+(kg*CLalf*ude_c*V_cruise_array_kn)/(498*W_S)
    n_cruise_neg = 1+(-1*kg*CLalf*ude_c*V_cruise_array_kn)/(498*W_S)
    n_dive_pos = 1+(kg*CLalf*ude_d*V_dive_array_kn)/(498*W_S)
    n_dive_neg = 1+(-1*kg*CLalf*ude_d*V_dive_array_kn)/(498*W_S)

    # Plot loads
    plt.figure(figsize=(8,4))
    plt.title(title1)
    plt.xlabel("V (ft/s)")
    plt.ylabel("n (-)")
    plt.plot(V[:upper_idx[0]],n_Stall[:upper_idx[0]], label='Stall', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(V[:lower_idx[0]],-n_Stall[:lower_idx[0]], label='Stall', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(V[upper_idx[0]:],n_pos[upper_idx[0]:], label='Limit Load Pos', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(V[lower_idx[0]:],n_neg[lower_idx[0]:], label='Limit Load Neg', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(Vd*np.ones(100),np.linspace(-nmin,nmax,100), label='Excess Speed', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(V_rough_array,n_rough_pos, label='Gust Rough', linestyle='dashdot', linewidth=2, marker=None, markersize=8)
    plt.plot(V_rough_array,n_rough_neg, label='Gust Rough', linestyle='dashdot', linewidth=2, marker=None, markersize=8)
    plt.plot(V_cruise_array,n_cruise_pos, label='Gust Cruise', linestyle='--', linewidth=2, marker=None, markersize=8)
    plt.plot(V_cruise_array,n_cruise_neg, label='Gust Cruise', linestyle='--', linewidth=2, marker=None, markersize=8)
    plt.plot(V_dive_array,n_dive_pos, label='Gust Dive', linestyle='--', linewidth=2, marker=None, markersize=8)
    plt.plot(V_dive_array,n_dive_neg, label='Gust Dive', linestyle='--', linewidth=2, marker=None, markersize=8)
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.05,1))
    plt.tight_layout()
    plt.show()

    # Plot combined V-n Diagram
    plt.figure(figsize=(8,4))
    plt.title(title2)
    plt.xlabel("V (ft/s)")
    plt.ylabel("n (-)")
    plt.plot(V[:upper_idx[0]],n_Stall[:upper_idx[0]], color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(V[:lower_idx[0]],-n_Stall[:lower_idx[0]], color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(np.linspace(V[upper_idx[0]],V_rough_array[-1],100),np.maximum(np.linspace(n_Stall[upper_idx[0]],n_rough_pos[-1],100),nmax),\
            color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(np.linspace(V[lower_idx[0]],V_rough_array[-1],100),np.minimum(np.linspace(-n_Stall[lower_idx[0]],n_rough_neg[-1],100),-nmin),\
            color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(np.linspace(V_rough_array[-1],V_cruise_array[-1],100),np.maximum(np.linspace(n_rough_pos[-1],n_cruise_pos[-1],100),nmax),\
             color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(np.linspace(V_rough_array[-1],V_cruise_array[-1],100),np.minimum(np.linspace(n_rough_neg[-1],n_cruise_neg[-1],100),-nmin),\
            color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(np.linspace(V_cruise_array[-1],V[-1],100),np.maximum(np.linspace(n_cruise_pos[-1],nmax,100),nmax),\
             color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(np.linspace(V_cruise_array[-1],V[-1],100),np.minimum(np.linspace(n_cruise_neg[-1],-nmin,100),-nmin),\
             color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.plot(Vd*np.ones(100),np.linspace(-nmin,nmax,100), color='black', linestyle='-', linewidth=2, marker=None, markersize=8)
    plt.grid(True)
    plt.show()

# V-n diagram for MTOW
W = 67551 # weight (lbf)
nmax = 3.3 # positive limit load
nmin = 1.2 # negative limit load
rho = 0.00237 # air density (slug/ft^3)
CL = 3.3 # max lift coefficient
S = 805.6 # wing area (ft^2)
Vc_ktas = 350 # KTAS
cbar = 9.51 # mean chord (ft)
CLalf = 5.73 # Lift slope (rad^-1)
v_n(W,nmax,nmin,rho,CL,S,Vc_ktas,cbar,CLalf,'Loads (MTOW)','V-n Diagram (MTOW)')

# V-n diagram for Minimum Weight
W = 48441 # weight (lbf)
nmax = 3.3 # positive limit load
nmin = 1.2 # negative limit load
rho = 0.00237 # air density (slug/ft^3)
CL = 3.3 # max lift coefficient
S = 805.6 # wing area (ft^2)
Vc_ktas = 350 # KTAS
cbar = 9.51 # mean chord (ft)
CLalf = 5.73 # Lift slope (rad^-1)
v_n(W,nmax,nmin,rho,CL,S,Vc_ktas,cbar,CLalf,'Loads (Minimum Weight)','V-n Diagram (Minimum Weight)')