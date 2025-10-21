clear
clc
close all

theta = 30*pi/180;
R = 0.375;
dxd = 0.001:0.001:1;
ALC = dxd;
sumomega = 4*568;
u = tan(theta)./(ALC/(sumomega*R)+dxd);
xdot = u*cos(theta) + u*tan(theta)*sin(theta);

plot(dxd,xdot)