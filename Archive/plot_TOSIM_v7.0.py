#!/usr/bin/python
import plotting as P
import mio as M
from pdf import *
import os
import sys
import matplotlib.pyplot as plt
import sixdof as SIX

if len(sys.argv) > 1:
    SIMULATE = int(sys.argv[1])
else:
    print('Need SIMULATE flag. Put a 0 or 1 if you want to simulate or not. Defaulting to 0')
    SIMULATE = 0
    
if SIMULATE == 1:
    ##Compile the code
    os.system('make')
    ##Run the code
    #os.system('./LinuxRun.exe Input_Files/Tether_Drop_Files/TOMAD.ifiles')
    #os.system('./LinuxRun.exe Input_Files/Forward_Flight/TOMAD.ifiles')
    #os.system('./LinuxRun.exe Input_Files/Forward_Flight_Steady/TOMAD.ifiles')
    #os.system('./LinuxRun.exe Input_Files/Hovering/TOMAD.ifiles')
    os.system('./Simulation.exe Input_Files/Hovering/TOSIM.ifiles')
    #os.system('./Simulation.exe Input_Files/Forward_Truck/TOSIM.ifiles')
else:
    print('Skipping Simulation and just plotting')

#Kick off pdf saver
pp = PDF(0,plt)

state_data = np.loadtxt('Output_Files/State.OUT')
[r,c] = np.shape(state_data)
print('Rows,Cols = ',r,c)

fontSize = 14

###Ok so the State.OUT file has a very large number of columns. Let's see what we got
#The first column is time
time = state_data[:,0]
#The rest is state and statedot
#The number of states is 13 + 12 + 7*NBEADS + 1 + 8
#Ok so the first 1:13 is the towed system state
towed_state = state_data[:,1:14] #0 is time and then 1:14 is towed
ylabel = ['x (ft)','y (ft)','z (ft)','q0','q1','q2','q3','u (ft/s)','v (ft/s)','w (ft/s)','p (rad/s)','q (rad/s)','r (rad/s)']
for idx in range(0,13):
    print('Plotting Towed State = ',ylabel[idx])
    plti = P.plottool(fontSize,'Time(sec)',ylabel[idx],'Towed')
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

#The next 14:25 is the copter states
copter_state = state_data[:,14:26]
ylabelQ = ['x (ft)','y (ft)','z (ft)','Roll Angle (deg)','Pitch Angle (deg)','Yaw Angle (deg)','u (ft/s)','v (ft/s)','w (ft/s)','p (rad/s)','q (rad/s)','r (rad/s)']
for idx in range(14,26):
    print('Plotting Copter State = ',ylabelQ[idx-14])
    plti = P.plottool(fontSize,'Time(sec)',ylabelQ[idx-14],'Quadcopter')
    #print ylabelQ[idx-14][-2]
    if ylabelQ[idx-14][-2] == 'g':
        factor = 180.0/np.pi
        print('Plotting Euler Angles')
    else:
        factor = 1.0
    plti.plot(time,copter_state[:,idx-14]*factor,'k-',linewidth=2)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()
#The next 26:end-8 are the tether states 7*NBEADS + 1
#Remember if you have 3 beads you have 3 position states and 3 velocity states per bead so that's
#6*NBEADS. But then you have NBEADS+1 tension states so you have
#6*NBEADS + NBEADS + 1 = 7*NBEADS + 1 the states go x,y,z,xdot,ydot,zdot for each bead and then tension
#is at the end
tether_state = state_data[:,26:-8]
ylabelTHR = ['X','Y','Z','XDOT','YDOT','ZDOT']
NBEADS = 0
beads = np.arange(0,NBEADS,1)
for n in beads:
    for idx in range(0,6): #Let's just plot the first bead
        print('Plotting Tether State = ',ylabelTHR[idx])
        plti = P.plottool(fontSize,'Time(sec)',ylabelTHR[idx],'Tether')
        plti.plot(time,tether_state[:,6*n+idx],'k-',linewidth=2)
        pp.savefig()
for n in beads:
    print('Plotting Tether State = T',n)
    plti = P.plottool(fontSize,'Time(sec)','T','Tether')
    plti.plot(time,tether_state[:,6*NBEADS+n],'k-',linewidth=2)
    pp.savefig()
#The final 8 states are the new quadcopter thrust states which I don't have plotted yet

#I have found that it's easier to plot the misc file so we'll plot that instead
misc_data = np.loadtxt('Output_Files/Misc.OUT')
[r,c] = np.shape(misc_data)
print('Rows,Cols = ',r,c)
tension = misc_data[:,NBEADS*7+5] #the plus 5 comes from t,vx,vy,vz
print('Plotting Tether State = T',0)
plti = P.plottool(fontSize,'Time(sec)','T (lbs)','Tether')
plt.gcf().subplots_adjust(left=0.18)
plti.plot(time,tension,'k-',linewidth=2)
pp.savefig()

#Look at delta x Towed and quad
towed_x = towed_state[:,0]
quad_x = copter_state[:,0]
delx = quad_x - towed_x

print('Plotting Towed - Quad Position')
plti = P.plottool(fontSize,'Time(sec)','Delta X (ft)','Quadcopter - Towed')
plt.gcf().subplots_adjust(left=0.18)
plti.plot(time,delx,'k-',linewidth=2)
pp.savefig()

##Control Out File
control_data = np.loadtxt('Output_Files/Controls.OUT')
[r,c] = np.shape(control_data)
print('Rows,Cols = ',r,c)
time_control = control_data[:,0]
#aileron = control_data[:,1]
#elevator = control_data[:,2]
#rudder = control_data[:,3]
#flaps = control_data[:,4]
controlY = ['Aileron (deg)','Elevator (deg)','Rudder (deg)','Flaps (deg)']
for i in range(1,5):
    print('Plotting = ',controlY[i-1])
    plti = P.plottool(fontSize,'Time(sec)',controlY[i-1],'Towed Control Deflections')
    plt.gcf().subplots_adjust(left=0.20)
    plti.plot(time,control_data[:,i]*180.0/np.pi,'k-',linewidth=2)
    pp.savefig()
pp.close()
