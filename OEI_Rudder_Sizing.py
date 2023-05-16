#OEI Rudder Sizing
def OEI_RudderSizing():
    
    N_t_crit = T_To_e*y_t

    #Drag-indued yawing moment
    N_D = 0.10*N_t_crit         #for variable pitch prop
    V_mc = 1.3*V_stall          #Minimum Control Speed