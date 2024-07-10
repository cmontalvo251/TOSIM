#!/usr/bin/python

import plotting as P
import mio as M
from pdf import *
import os
import sys
import matplotlib.pyplot as plt

if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    print 'Need filename'
    sys.exit()

#Kick off pdf saver
pp = PDF(0,plt)

exp_data = M.dlmread(filename,delimiter=' ')

fontSize = 12

###Ok so the State.OUT file has a very large number of columns. Let's see what we got
#The first column is time
time = exp_data[:,0]

ylabels = ['Time (sec)','Roll (rad)','Roll Filtered (rad)','Pitch (rad)','Pitch Filtered (rad)','Yaw (rad)','Yaw Filtered (rad)','Roll Rate (rad/s)','Pitch Rate (rad/s)','Yaw Rate (rad/s)','Aileron (deg)','Elevator (deg)','Rudder (deg)']

#Plot Roll and Roll Filtered
plti = P.plottool(fontSize,'Time(sec)','Roll Angle (deg)','')
plti.plot(time,exp_data[:,1]*180./np.pi,'k-',linewidth=2,label='Raw Signal')
plti.plot(time,exp_data[:,2]*180./np.pi,'k--',linewidth=2,label='Filtered')
plti.legend()
pp.savefig()

#Pitch
plti = P.plottool(fontSize,'Time(sec)','Pitch Angle (deg)','')
plti.plot(time,exp_data[:,3]*180./np.pi,'k-',linewidth=2,label='Raw Signal')
plti.plot(time,exp_data[:,4]*180./np.pi,'k--',linewidth=2,label='Filtered')
plti.legend()
pp.savefig()

#Yaw
plti = P.plottool(fontSize,'Time(sec)','Yaw Angle (deg)','')
plti.plot(time,exp_data[:,5]*180./np.pi,'k-',linewidth=2,label='Raw Signal')
plti.plot(time,exp_data[:,6]*180./np.pi,'k--',linewidth=2,label='Filtered')
plti.legend()
pp.savefig()

#Control Effort
plti = P.plottool(fontSize,'Time(sec)','Control Effort (deg)','')
plti.plot(time,exp_data[:,10],'k-',linewidth=2,label='Aileron')
plti.plot(time,exp_data[:,11],'k--',linewidth=2,label='Elevator')
plti.plot(time,exp_data[:,12],'k-.',linewidth=2,label='Rudder')
plti.legend()
pp.savefig()

# for idx in range(0,13):
#     print(ylabels[idx])
#     plti = P.plottool(fontSize,'Time(sec)',ylabels[idx]+str(idx),'')
#     plti.plot(time,exp_data[:,idx],'k-',linewidth=2)
#     pp.savefig()
    
pp.close()


###2_21_2018_MedalofHonor - all garbage
###Archive - all garbage
###Not sure - Organized files based on GPS so not sure on these files in here

###Files in 2_27_2018 are definitely from Muni Park because GPS says so and we only went there one day

###FAST012.TXT - 5 second file but has GPS - 30.70827293 -88.16120147 - MUNICIPAL PARK!!!
###FAST013.TXT - 27 second file but has GPS - 30.70827293 -88.16120147 - MUNICIPAL PARK!!!
###FAST014.TXT - 296 second file AND has GPS - 30.70827293 -88.16120147 - MUNICIPAL PARK!!! - DEFINITELY WORTH A SECOND LOOK!!!
###FAST015.TXT - 60 second file no GPS
###FAST016.TXT - 38 second file no GPS
###FAST017.TXT - 290 second file AND HAS GPS - 30.70821571 -88.16117858 - MPARK - LOOK
###FAST018.TXT - 57 second file no GPS
