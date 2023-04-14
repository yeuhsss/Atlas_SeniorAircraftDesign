#Refined Weight Estimate
import numpy as np
#Too many Variables
#Wing
W_TO = 82500   #MTOW (iterate on)
P_rshp = 2380   #iterable

def calcEmptyWeight(W_TO, P_rshp):
    W_dg = W_TO         #design weight
    N_z = 1.5*3.5          #ult load factor, raymor table 14.2
    S_w = 805.06257
    A = 10.0613
    t_c_root = 0.15450
    lam = .7525
    Lam = 20    #degrees, check rads
    S_csw = 2*((4.254*9.024*2) + (6.295*(5.449 + 5.802)))        #wing control surface area, upper and lower

    #HT
    K_uht = 1.143
    F_w = 10
    B_h = 31.6
    S_ht = 202.18
    L_t = 24.5        #estimated length, wing MAC to tail MAC
    K_y = 0.3*L_t
    Lam_ht = 20
    A_h = 5.6
    S_e = 2*((1.350+3.020)*0.5*8.478)   #elevator area

    #VT
    Ht_Hv = 1
    S_vt = 190.79
    K_z = L_t
    Lam_vt = 40 #About a 40deg sweep
    A_v = 70.4
    t_c_root_vt =  0.16512 #check with tre

    #fuselage
    B_w = 90
    K_door = 1.12
    K_Lg = 1.12
    L = 73    #measured aribtrary length, excludes nose cap and radome cowling 
    S_f = 2093
    df =10*12   #diamaeter of fuselage, in
    D = 0.02*df + 1     # fuselage structrual depth in

    #LG_m
    K_mp = 1.15
    W_l = 0.043*W_dg    #Raymor approximation, Table 15.2
    N_gear = 3
    N_l = 1.5*N_gear
    L_m = 3.68*12   #open vsp measure, (in)
    N_mw = 4
    N_mss = 3
    V_stall = 205.913

    #LG_n
    K_np = 1.15
    L_n = 3.771*12  #open vsp measure, (in)
    N_nw = 2

    #enginecontrol
    N_en = 2
    L_ec = 31

    #flightcontrol
    N_f = 5
    N_m = 1
    S_r = 2*(2*(1.033+2.288)*0.5*10.691)    #rudder area
    S_cs = S_csw + S_r + S_e
    I_yaw = 2494172.151 #simulated using const density in openvsp

    #fuelsys
    V_t = 128.604   #total volume
    V_i = 128.604   #integral fuel tank volume (wings)
    V_p = 128.604   #self-sealing, "protexted tank volume"
    N_t = 4         #number of tanks (4)

    #avionics
    W_uav = 1100 #keep for now, look into typical uninstalled avionics 

    #instruments
    K_r = 1.0
    K_tp = 0.793
    N_c = 4
    N_en = 2
    L_f = 81

    #hydraulics
    #already defined

    #Air Conditioning   #readdress later
    N_p = 53
    #V_pr =         #need internal pressurized volume, too complex for now

    #nacelle group
    K_ng = 1.15
    N_Lt = 14.167
    N_w = 2.354     #ft, EXTREMELY SMALL, MAY NEED RESIZING
    S_n = 52.454

    W_engine = P_rshp**(0.9306)*10**(-0.1205)  #Weight of engine, lbs

    K_p = 1.4
    K_tr = 1.0


    #=======================
    #STRUCTURAL WEIGHT
    #=======================

    #Wing Weight
    W_wing = 0.0051*(W_dg*N_z)**0.557*S_w**0.649*A**0.5*(t_c_root)**-0.4*(1 + lam)**0.1*np.cos(Lam/180.0*np.pi)**-1.0*S_csw**0.1
    #Horizontal Tail Weight
    W_HT = 0.0379*K_uht*(1 + F_w/B_h)**-0.25*W_dg**0.639*N_z**0.10*S_ht**0.75*L_t**-1.0*K_y**0.704*np.cos(Lam_ht)**-1.0*A_h**0.166*(1 + S_e/S_ht)**0.1
    #Vertical Tail Weight
    W_VT = 0.0026*(1 + Ht_Hv)**0.225*W_dg**0.556*N_z**0.536*L_t**(-0.5)*S_vt**0.5*K_z**0.875*np.cos(Lam_vt)**(-1.0)*A_v**0.35*(t_c_root_vt)**(-0.5)

    #K_ws needed for W_fuse
    K_ws = 0.75*((1 + 2*lam)/(1 + lam))*(B_w*np.tan(Lam/L))

    #Fuselage Weight
    W_fuse = 0.3280*K_door*K_Lg*(W_dg*N_z)**0.5*L**0.25*S_f**0.302*(1 + K_ws)**0.04*(L/D)**0.10
    #Main Landing Gear Weight
    W_lg_main = 0.0106*K_mp*W_l**0.888*N_l**0.25*L_m**0.4*N_mw**0.321*N_mss**-0.5*V_stall**0.1
    #Nose Landing Gear Weight
    W_lg_nose = 0.032*K_np*W_l**0.646*N_l**0.2*L_n**0.5*N_nw**0.45

    #=======================
    #CONTROL SYSTEM WEIGHT
    #=======================
    #Engine Controls Weight
    W_encl = 5.0*N_en + 0.80*L_ec

    #Flight Controls Weight
    W_fc = 145.9*N_f**0.554*(1 + N_m/N_f)**-1.0*S_cs**0.20*(I_yaw*10**-6)**0.07

    #=======================
    #Systems Weight
    #=======================
    #Fuel System Weight
    W_fs = 2.405*V_t**0.606*(1 + V_i/V_t)**-1.0*(1 + V_p/V_t)*N_t**0.5

    #Avionics Weight
    W_av = 1.73*W_uav**0.983

    #Instruments Weight
    W_instr = 4.509*K_r*K_tp*N_c**0.541*N_en*(L_f + B_w)**0.5

    #Hydraulics or electronics Weight (tbd)
    W_hyd = 0.2673*N_f*(L_f + B_w)**0.5

    #Anti-icing
    W_ai = 0.002*W_dg

    #Air Conditionting
    #W_ac = 63.36*N_p**0.25*(V_pr/1000)**0.604*W_uav**0.10      #holding off for now, need a way to find pressurized volume

    #=======================
    #Propulsion Weight
    #=======================
    #Needed for nacelle group
    W_ec = 2.331*W_engine**0.901*K_p*K_tr

    #Nacelle Group Weight
    W_ng = .6724*K_ng*N_Lt**0.10*N_w**0.294*N_z**0.119*W_ec**0.611*N_en**0.984*S_n**0.224


    W_empty = W_ng + W_ai + W_hyd + W_instr + W_av + W_fs + W_fc + W_encl + W_lg_nose + W_lg_main + W_fuse +  W_HT + W_wing + W_VT

    return W_empty

yo  = calcEmptyWeight(W_TO, P_rshp)
print(yo)