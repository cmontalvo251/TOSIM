#!/usr/bin/python

import plotting as P
import mymath as MM
import mio as M
from pdf import *
import os
import sys
import matplotlib.pyplot as plt

#if len(sys.argv) > 1:
#    filename = sys.argv[1]
#else:
#    print 'Need filename'
#    sys.exit()

#Kick off pdf saver
pp = PDF(0,plt)

fontSize = 14

##This file is good. Control is on 
###FAST014.TXT - 296 second file AND has GPS - 30.70827293 -88.16120147 - MUNICIPAL PARK!!! - DEFINITELY WORTH A SECOND LOOK!!!
##This is good too. More than likely changed some gains
###FAST017.TXT - 290 second file AND HAS GPS - 30.70821571 -88.16117858 - MPARK - LOOK

##Where to get uncontrolled???
##Try and find a file where it looks like we hit a puddle
#FAST007.TXT and FAST010.TXT didn't get GPS but they are very large files
#FAST007 looks like we hit a puddle
#FAST010 must have been debugging after we hit the puddle.

##########CURRENT CONSENSUS#############33

##FAST007 must be uncontrolled

filename = 'FAST007.TXT'
#For entire trajectory
tstart = 1.
tend = 265.
#For parts that are in the air before the crash
# tstart = 125.
# tend = 225.
#For good plotting parts
tstart = 160.
tend = 185.

##FAST014 must be low gain

filename = 'FAST014.TXT'
# #For GPS plots
#tstart = 110.
#tend = 220.
# #For controlled plots
tstart = 165.
tend = 180.

##FAST017 must be high gain

#########################################

exp_data = M.dlmread(filename,delimiter=' ')

###Ok so the State.OUT file has a very large number of columns. Let's see what we got
#The first column is time
time = exp_data[:,0]

##Extract a start an end point to plot
loc = np.where(time > tstart)[0]
loc_start = loc[0]
print('Start Point = ',loc_start)
loc = np.where(time > tend)[0]
loc_end = loc[0]
print('End Point = ',loc_end)

ylabels = ['Time (sec)','GPS FIX','Lat (Deg)','Lon (Deg)','Speed GPS (kts)','GPS Angle (deg)','Altitude GPS (m)','Roll (rad)','Pitch (rad)','Yaw (rad)','Roll Rate (rad/s)','Pitch Rate (rad/s)','Yaw Rate (rad/s)','Aileron (deg)','Elevator (deg)','Rudder (deg)']

# for idx in range(0,len(ylabels)):
#     print(ylabels[idx])
#     plti = P.plottool(fontSize,'Time(sec)',ylabels[idx]+str(idx),'')
#     plti.plot(time[loc_start:loc_end],exp_data[loc_start:loc_end,idx],'k-',linewidth=2)
#     pp.savefig()

latitude = exp_data[loc_start:loc_end,2]
longitude = exp_data[loc_start:loc_end,3]

for idx in range(1,len(latitude)):
    if abs(latitude[idx]-latitude[idx-1]) > 2:
        latitude[idx] = latitude[idx-1]
    if abs(longitude[idx]-longitude[idx-1]) > 2:
        longitude[idx] = longitude[idx-1]

plti = P.plottool(10,'Latitude (deg)','Longitude (deg)','Latitude vs. Longitude')
plti.plot(latitude,longitude,'k-',linewidth=1,label='Ground Track')
plti.plot(latitude[0],longitude[0],'ks',linewidth=1,label='Begin')
plti.plot(latitude[-1],longitude[-1],'k*',linewidth=1,label='End')
#plti.plot(latitude[0],longitude[0],'ks',linewidth=1,label='Landing Location')
#plti.plot(latitude[-1],longitude[-1],'k*',linewidth=1,label='Take Off Location')
plti.legend()
pp.savefig()

##Filter Data
a = 8.
#a = 25.

gps_speed = exp_data[loc_start:loc_end,4]
outY,outX = MM.LowPass(exp_data[loc_start:loc_end,4],time[loc_start:loc_end],4.)
plti = P.plottool(fontSize,'Time(sec)','GPS Speed (knots)','')
plti.plot(time[loc_start:loc_end],gps_speed,'k-',linewidth=2)
#plti.plot(time[loc_start:loc_end],outY,'k-',linewidth=2)
pp.savefig()

#Plot Roll
plti = P.plottool(fontSize,'Time(sec)','Roll Angle (deg)','')
#plti.plot(time[loc_start:loc_end],exp_data[loc_start:loc_end,7]*180./np.pi,'k-',linewidth=2)
outY,outX = MM.LowPass(exp_data[loc_start:loc_end,7]*180./np.pi,time[loc_start:loc_end],a)
plti.plot(time[loc_start:loc_end],outY,'k-',linewidth=2)
pp.savefig()

#Pitch
plti = P.plottool(fontSize,'Time(sec)','Pitch Angle (deg)','')
#plti.plot(time[loc_start:loc_end],exp_data[loc_start:loc_end,8]*180./np.pi,'k-',linewidth=2)
outY,outX = MM.LowPass(exp_data[loc_start:loc_end,8]*180./np.pi,time[loc_start:loc_end],a)
plti.plot(time[loc_start:loc_end],outY,'k-',linewidth=2)
pp.savefig()

#Yaw
plti = P.plottool(fontSize,'Time(sec)','Yaw Angle (deg)','')
#plti.plot(time[loc_start:loc_end],exp_data[loc_start:loc_end,9]*180./np.pi,'k-',linewidth=2)
outY,outX = MM.LowPass(exp_data[:,9]*180./np.pi,time,a)

#time_plot = time[loc_start:loc_end]
yaw_plot = outY

##I want to cut out 165 to 170 and just connect with a line
loc = np.where(time > 168.6)[0]
_start = loc[0]
loc = np.where(time > 170.)[0]
_end = loc[0]

m = (yaw_plot[_end]-yaw_plot[_start])/(time[_end]-time[_start])

#First order filter
s = 0.2

#for x in range(_start,_end):
#    yaw_plot[x+1] = (1-s)*yaw_plot[x] + s*yaw_plot[_end]

plti.plot(time[loc_start:loc_end],yaw_plot[loc_start:loc_end],'k-',linewidth=2)
pp.savefig()

##Try and Unwrap Yaw
# yaw = exp_data[loc_start:loc_end,9]*180./np.pi
# yaw_unwrap = MM.unwrap_complex(yaw)
# plti = P.plottool(fontSize,'Time(sec)','Yaw Angle (deg)','')
# plti.plot(time[loc_start:loc_end],yaw_unwrap,'k-*',linewidth=2)
# pp.savefig()

# #Control Effort
plti = P.plottool(fontSize,'Time(sec)','Aileron (deg)','')
plti.plot(time[loc_start:loc_end],exp_data[loc_start:loc_end,13]-90.,'k-',linewidth=2,label='Aileron')
pp.savefig()
plti = P.plottool(fontSize,'Time(sec)','Elevator (deg)','')
plti.plot(time[loc_start:loc_end],exp_data[loc_start:loc_end,14]-90.,'k-',linewidth=2,label='Elevator')
pp.savefig()
plti = P.plottool(fontSize,'Time(sec)','Rudder (deg)','')
plti.plot(time[loc_start:loc_end],exp_data[loc_start:loc_end,15]-90.,'k-',linewidth=2,label='Rudder')
#plti.legend()
pp.savefig()

pp.close()

###2_21_2018_MedalofHonor - all garbage
###Archive - all garbage
###Not sure - Organized files based on GPS so not sure on these files in here

###Files in 2_27_2018 are definitely from Muni Park because GPS says so and we only went there one day
###FAST012.TXT - 5 second file but has GPS - 30.70827293 -88.16120147 - MUNICIPAL PARK!!!
###FAST013.TXT - 27 second file but has GPS - 30.70827293 -88.16120147 - MUNICIPAL PARK!!!
###FAST015.TXT - 60 second file no GPS
###FAST016.TXT - 38 second file no GPS
###FAST018.TXT - 57 second file no GPS
