#!/usr/bin/python3

import sys
sys.path.append('Supplemental_Files')
import plotting as P
import mio as M
from pdf import *
import os
import matplotlib.pyplot as plt
import sixdof as SIX
from mpl_toolkits.mplot3d import Axes3D
from celluloid import Camera

state_data = np.loadtxt('Output_Files/State.OUT')
[r,c] = np.shape(state_data)
print('Rows,Cols = ',r,c)

###Ok so the State.OUT file has a very large number of columns. Let's see what we got
#The first column is time
time = state_data[:,0]
#The rest is state and statedot
#The number of states is 12 + 21 + 7*NBEADS + 1
#Ok so the first 1:12 is the towed system state but in python you add one at the end
driver_state = state_data[:,1:13] #0 is time and then 1:12 is driver but you add 1 at the end for Python
ylabel = ['x (ft)','y (ft)','z (ft)','Roll Angle (deg)','Pitch Angle (deg)','Yaw Angle (deg)','u (ft/s)','v (ft/s)','w (ft/s)','p (rad/s)','q (rad/s)','r (rad/s)']

#The next 13:33 is the copter states
towed_state = state_data[:,13:34] #again remember you add one
ylabelT = ['x (ft)','y (ft)','z (ft)','q0','q1','q2','q3','u (ft/s)','v (ft/s)','w (ft/s)','p (rad/s)','q (rad/s)','r (rad/s)','T1 (N)','T1 dot (N/s)','T2 (N)','T2 dot (N/s)','T3 (N)','T3 dot (N/s)','T4 (N)','T4 (N/s)']

#Need to convert Quaternions to Euler Angles
quats = towed_state[:,3:7]
q0123 = [quats[:,0],quats[:,1],quats[:,2],quats[:,3]]
euler_angles = SIX.quat2euler(q0123)
ylabelEULER = ['Roll Angle (deg)','Pitch Angle (deg)','Yaw Angle (deg)']

#The next 34:end are the tether states 7*NBEADS + 1
#Remember if you have 3 beads you have 3 position states and 3 velocity states per bead so that's
#6*NBEADS. But then you have NBEADS+1 tension states so you have
#6*NBEADS + NBEADS + 1 = 7*NBEADS + 1 the states go x,y,z,xdot,ydot,zdot for each bead and then tension
#is at the end
tether_state = state_data[:,34:]
ylabelTHR = ['X','Y','Z','XDOT','YDOT','ZDOT']
NBEADS = 10

##ANIMATION ROUTINE
fig = plt.figure('3-D')
camera = Camera(fig)
ax = fig.add_subplot(111,projection='3d')
totalplottingpts = 100 ##this restricts us to a certain number of points  (lower number is smoother animation)
##So no we need the skip parameter
skip = int(float(len(time))/float(totalplottingpts))

def figparams(x,y,z):
    fx = 20
    fy = 20
    fzl = -2
    fzu = 40
    ax.set_xlim([x-fx,x+fx])
    ax.set_ylim([y-fy,y+fy])
    ax.set_zlim([z+fzl,z+fzu])
    plt.grid()
def draw_cube(x,y,z,phi,theta,psi,l,w,h,axin,color):
    #Bottom Face
    x = np.array([-0.5,0.5,0.5,-0.5])*l + x
    y = np.array([-0.5,-0.5,0.5,0.5])*w + y
    z = (np.array([0,0,0,0])-0.5)*h/2.0 - z
    verts = [list(zip(x,y,z))]
    ax.add_collection3d(Poly3DCollection(verts))
def draw_line(x1,y1,z1,x2,y2,z2):
    plt.plot([x1,x2],[y1,y2],[z1,z2])
def decimal(x,n):
    return float(str(x)[:str(x).find('.')+(n+1)])
def getfilename(i,L):
    numdx = '0'*L
    istr = str(i)
    filename = 'Frames/'+numdx[0:L-len(istr)]+istr+'.png'
    return filename
iplot = 0
for i in range(0,len(time)):
    if i >= iplot:
        plt.pause(0.01)
        plt.cla()
        #FIRST DRAW THE TRUCK
        xd = driver_state[i,0]
        yd = driver_state[i,1]
        zd = driver_state[i,2]
        figparams(xd,yd,zd)
        draw_cube(xd,yd,zd,0,0,0,10,10,0.1,ax,[1,0,0])
        ###NOW DRAW THE TOWED
        xt = towed_state[i,0]
        yt = towed_state[i,1]
        zt = towed_state[i,2]
        draw_cube(xt,yt,zt,0,0,0,2,2,0.1,ax,[0,1,0])
        ##NOW DRAW THE BEADS
        for n in range(0,NBEADS):
            xb = tether_state[i,6*n+0]
            yb = tether_state[i,6*n+1]
            zb = tether_state[i,6*n+2]
            ax.scatter(xb,yb,-zb,s=20)
            #draw a line to the previous bead
            if n > 0:
                draw_line(xb,yb,-zb,xprev,yprev,-zprev)
            xprev = xb
            yprev = yb
            zprev = zb
        #Draw a line from the truck to the first bead
        draw_line(xd,yd,-zd,tether_state[i,0],tether_state[i,1],-tether_state[i,2])
        #Then draw a line from the last bead to the towed system
        draw_line(xt,yt,-zt,tether_state[i,6*(NBEADS-1)+0],tether_state[i,6*(NBEADS-1)+1],-tether_state[i,6*(NBEADS-1)+2])
        #Then clean up some shit
        ax.set_title(decimal(time[i],1))
        iplot+=skip
        print(i,' out of ',len(time))
        filename = getfilename(i,5)
        plt.savefig(filename,format='png')
        
