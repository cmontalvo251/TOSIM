clear
clc
close all

theta = 30*pi/180;
R = 0.375;
dxd = 0.001:0.0001:1;
ALC = 1;
sumomega = 3031.0459660189467; %4*568;
u = tan(theta)./(ALC/(sumomega*R)+dxd);
xdot = u*cos(theta) + u*tan(theta)*sin(theta);

uvec = linspace(52,52,length(dxd));

figure()
plot(dxd,xdot)
hold on
plot(dxd,uvec,'r--')
