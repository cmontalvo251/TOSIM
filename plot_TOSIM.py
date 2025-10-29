#!/usr/bin/python3

import sys
sys.path.append('Supplemental_Files')
import plotting as P
import mio as M
from pdf import *
import os
import matplotlib.pyplot as plt
import sixdof as SIX

if len(sys.argv) > 1:
    SIMULATE = int(sys.argv[1])
else:
    print('Need SIMULATE flag. Put a 0 or 1 if you want to simulate or not. Defaulting to 0')
    SIMULATE = 0
    
if SIMULATE > 0:
    ##Compile the code
    os.system('make')
    ##Run the code
    #os.system('./LinuxRun.exe Input_Files/Tether_Drop_Files/TOMAD.ifiles')
    #os.system('./LinuxRun.exe Input_Files/Forward_Flight/TOMAD.ifiles')
    #os.system('./LinuxRun.exe Input_Files/Forward_Flight_Steady/TOMAD.ifiles')
    #os.system('./LinuxRun.exe Input_Files/Hovering/TOMAD.ifiles')
    #os.system('./Simulation.exe Input_Files/Hovering/TOSIM.ifiles')
    #os.system('./Simulation.exe Input_Files/Forward_Truck/TOSIM.ifiles')
    os.system('./Simulation.exe Input_Files/Helicopter_Towing_Ball/TOSIM.ifiles')

    if SIMULATE > 1:
        sys.exit()
else:
    print('Skipping Simulation and just plotting')

#Kick off pdf saver
pp = PDF(0,plt)

state_data = np.loadtxt('Output_Files/State.OUT')
[r,c] = np.shape(state_data)
print('Rows,Cols = ',r,c)

fontSize = 10

###Ok so the State.OUT file has a very large number of columns. Let's see what we got
#The first column is time
time = state_data[:,0]
#The rest is state and statedot
#The number of states is 12 + 21 + 7*NBEADS + 1
#The first 12 are the driver states
#The next 21 are the quad states including T and Tdots
#the final ones are the beads

#The next 13:33 is the copter states
towed_state = state_data[:,13:34] #again remember you add one
ylabelT = ['x (ft)','y (ft)','z (ft)','q0','q1','q2','q3','u (ft/s)','v (ft/s)','w (ft/s)','p (rad/s)','q (rad/s)','r (rad/s)','T1 (N)','T1 dot (N/s)','T2 (N)','T2 dot (N/s)','T3 (N)','T3 dot (N/s)','T4 (N)','T4 dot (N/s)']
for idx in range(0,21):
    print('Plotting Towed State = ',ylabelT[idx])
    plti = P.plottool(fontSize,'Time(sec)',ylabelT[idx],'Towed')
    plti.plot(time,towed_state[:,idx],'k-',linewidth=2)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()

#Need to convert Quaternions to Euler Angles
quats = towed_state[:,3:7]
q0123 = [quats[:,0],quats[:,1],quats[:,2],quats[:,3]]
euler_angles = SIX.quat2euler(q0123)
ylabelEULER = ['Roll Angle (deg)','Pitch Angle (deg)','Yaw Angle (deg)']
for idx in range(0,3):
    print('Plotting Towed State = ',ylabelEULER[idx])
    plti = P.plottool(fontSize,'Time(sec)',ylabelEULER[idx],'Towed')
    plti.plot(time,euler_angles[idx]*180.0/np.pi,'k-',linewidth=2)
    plt.gcf().subplots_adjust(left=0.20)
    pp.savefig()

#The next 34:end are the tether states 7*NBEADS + 1
#Remember if you have 3 beads you have 3 position states and 3 velocity states per bead so that's
#6*NBEADS. But then you have NBEADS+1 tension states so you have
#6*NBEADS + NBEADS + 1 = 7*NBEADS + 1 the states go x,y,z,xdot,ydot,zdot for each bead and then tension
#is at the end
tether_state = state_data[:,34:]
ylabelTHR = ['X','Y','Z','XDOT (ft/s)','YDOT (ft/s)','ZDOT (ft/s)']
NBEADS = 1
for n in range(0,NBEADS):
    for idx in range(0,6): 
        print('Plotting Tether State = ',ylabelTHR[idx],' ',n)
        plti = P.plottool(fontSize,'Time(sec)',ylabelTHR[idx],'Tether')
        plti.plot(time,tether_state[:,6*n+idx],'k-',linewidth=2)
        pp.savefig()
for n in range(0,NBEADS+1): #There is 1 more tension state than beads
    print('Plotting Tether State = T',n)
    plti = P.plottool(fontSize,'Time(sec)','T','Tether')
    plti.plot(time,tether_state[:,6*NBEADS+n],'k-',linewidth=2)
    pp.savefig()

control_data = np.loadtxt('Output_Files/Controls.OUT')
[r,c] = np.shape(control_data)
print('Rows,Cols = ',r,c)

time_control = control_data[:,0]
ylabelControl = ['Throttle','Aileron','Elevator (rad)','Rudder','Flaps','PWM_quad_1','PWM_quad_2','PWM_quad_3','PWM_quad_4', 'σ_p', 'σ_q'] #Ω ω - idk if you want these
for n in range(0,11):
    print('Plotting Control STates = ',ylabelControl[n])
    plti = P.plottool(fontSize,'Time(sec)',ylabelControl[n],'Control')
    plti.plot(time_control,control_data[:,n+1],'k-',linewidth=2)
    pp.savefig()

#Ok so the first 1:12 is the towed system state but in python you add one at the end
driver_state = state_data[:,1:13] #0 is time and then 1:12 is driver but you add 1 at the end for Python
ylabel = ['x (ft)','y (ft)','z (ft)','Roll Angle (deg)','Pitch Angle (deg)','Yaw Angle (deg)','u (ft/s)','v (ft/s)','w (ft/s)','p (rad/s)','q (rad/s)','r (rad/s)']
for idx in range(0,12):
    print('Plotting Driver State = ',ylabel[idx])
    plti = P.plottool(fontSize,'Time(sec)',ylabel[idx],'Driver')
    #print('Plot created...')
    plti.plot(time,driver_state[:,idx],'k-',linewidth=2)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()

pp.close()
