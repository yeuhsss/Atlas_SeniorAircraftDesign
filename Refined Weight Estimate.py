#Refined Weight Estimate
import numpy as np

#STRUCTURAL WEIGHT
W_wing = 0.0051*(W_dg*N_z)**0.557*S_w**0.649*A**0.5*(t_c_root)**-0.4*(1 + lam)**0.1*np.cos(Lam/180.0*np.pi)**-1.0*S_csw**0.1

W_HT = 0.0379*K_uht*(1 + F_w/B_h)**-0.25*W_dg**0.639*N_z**0.10*S_ht**0.75*L_t**-1.0*K_y**0.704*np.cos(Lam_ht)**-1.0*A_h**0.166*(1 + S_e/S_ht)**0.1

W_VT = 0.0026*(1 + Ht_Hv)**0.225*W_dg**0.556*N_z**0.536*L_t**-0.5*S_vt**0.5*K_z**0.875*np.cos(Lam_vt)**-1.0*A_v**0.35(t_c_root_vt)**-0.5

#K_ws needed for W_fuse
K_ws = 0.75*((1 + 2*lam)/(1 + lam))*(B_w*np.tan(Lam/L))

W_fuse = 0.3280*K_door*K_Lg*(W_dg*N_z)**0.5*L**0.25*S_f**0.302*(1 + K_ws)**0.04*(L_D)**0.10

W_lg_main = 0.0106*K_mp*W_l**0.888*N_l**0.25*L_m**0.4*N_mw**0.321*N_mss**-0.5*V_stall**0.1

W_lg_nose = 0.032*K_np*W_l**0.646*N_l**0.2*L_n**0.5*N_nw**0.45

