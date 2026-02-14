import numpy as np


#Compute Vstall of the Aircraft
weight = 3.0 #lbf
rho = 0.00238 #slugs/ft^3
S = 3.65625	#ft^2
Cl0 = -0.07636788929236879
Cla = 5.185317204810547
u = 40.0 #ft/s
w = 9.0 #ft/s
AoA = np.arctan2(w,u) #radians
print('AoA (degrees) = ',AoA*180/np.pi)
pitch_ss = 12.0 #degrees
alpha_ss = pitch_ss * (np.pi/180) #radians
Clmax = Cl0 + Cla*alpha_ss
print('Clmax = ',Clmax)
Vstall = np.sqrt(2*weight/(rho*S*Clmax))
print('Vstall (ft/s) = ',Vstall)
print('VTO (ft/s) = ',1.2*Vstall)