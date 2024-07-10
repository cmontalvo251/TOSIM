#!/usr/bin/python

import mio as M
import plotting as P
import matplotlib.pyplot as plt
from pdf import *
import os
import sys

if len(sys.argv) > 1:
    SIMULATE = int(sys.argv[1])
else:
    print 'Need SIMULATE flag'
    sys.exit()

if SIMULATE == 1:
    ##Compile the code
    os.system('make')
    ##Run the code
    os.system('./LinuxRun.exe Input_Files/TOMAD.ifiles')
else:
    print 'Skipping Simulation and just plotting'

#Kick off pdf saver
pp = PDF(0,plt)

state_data = M.dlmread('Output_Files/State.OUT',delimiter=' ')

fontSize = 12

###Ok so the State.OUT file has a very large number of columns. Let's see what we got
#The first column is time
time_sim = state_data[:,0]
#The rest is state and statedot
#The number of states is 13 + 12 + 7*NBEADS + 1 + 8
#Ok so the first 1:13 is the towed system state
towed_state = state_data[:,1:14] #0 is time and then 1:13 is towed
ylabel = ['X','Y','Z','Q0','Q1','Q2','Q3','U','V','W','P','Q','R']
print('Plotting Towed State = ',ylabel[2])
plti = P.plottool(fontSize,'Time(sec)',ylabel[2],'Towed')
plti.plot(time_sim,towed_state[:,2],'k-',linewidth=2)
pp.savefig()

print('Plotting Towed State = ',ylabel[9])
plti = P.plottool(fontSize,'Time(sec)',ylabel[9],'Towed')
plti.plot(time_sim,towed_state[:,9],'k-',linewidth=2)
pp.savefig()

#The next 14:25 is the copter states
copter_state = state_data[:,14:26]
#The next 26:end-8 are the tether states 7*NBEADS + 1
#Remember if you have 3 beads you have 3 position states and 3 velocity states per bead so that's
#6*NBEADS. But then you have NBEADS+1 tension states so you have
#6*NBEADS + NBEADS + 1 = 7*NBEADS + 1 the states go x,y,z,xdot,ydot,zdot for each bead and then tension
#is at the end
tether_state = state_data[:,26:-8]
ylabelTHR = ['X','Y','Z','XDOT','YDOT','ZDOT']
NBEADS = 0
tension_simulation = tether_state[:,6*NBEADS]

##The data in this file was from experiment two on October 25th,2018
##The weight was 334 grams and was dropped the full length of the tether.
##The length of the tether was 13.5 inches
data = M.dlmread('Load_Cell_Test/Load_Cell_Test.csv')
plti = P.plottool(18,'Time(sec)','Load Cell Reading (lbf)','Tether Calibration')

time = data[:,2]-15-0.4-0.07
load_cell_reading = -data[:,0] ###this is in grams so we need to convert to lbf

load_cell_lbf = (load_cell_reading/1000.0)*2.2

##Plot experimental data
plti.plot(time,load_cell_lbf,'k--',label='Experimental Data',lineWidth=2)
##Plot simulation data
plti.plot(time_sim,tension_simulation,'k-',label='Simulation Data',lineWidth=2)
plt.xlim([0,1])
plt.legend()
pp.savefig()

pp.close()
