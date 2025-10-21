function DrawHeli(xc,yc,zc,phi,theta,psi,xt,yt,zt)

%%%%Helicopter Parameters
width = 0.3772966; %%% lphi x-direction
length = 0.656168; %%% ltheta y-direction
height = 0.19685;
lphi = 0.7020997;
ltheta = 0.4363517;
rotor_diameter = 0.375*2; %%%Feet
hub_z_distance = height/2;
hub_x_distance = ltheta;%width/2;
hub_y_distance = lphi;%length/2;

%%%Draw Tether Attachment Point
%%% Red
% CubeDraw(length,width,height,xt,yt,zt,phi,theta,psi,[1 0 0])

%%%%Draw Main Fuselage
%%% Blue
%CubeDraw(width,length,height,xc+length/4,yc,zc,phi,theta,psi,[0 0 1])

%%%%Draw Tail Boom
%%% Green
CubeDraw(length,width,height,xc,yc,zc,phi,theta,psi,[1 0 0])

%%%Draw Main Rotor Disk
[X1,Y1,Z1] = cylinder(rotor_diameter/2, 30);
X1 = X1 + xc - hub_x_distance;
Y1 = Y1 + yc + hub_y_distance;
Z1 = Z1*0.4 + zc*0.99 - hub_z_distance*2;
surf(X1,Y1,Z1)

[X2,Y2,Z2] = cylinder(rotor_diameter/2, 30);
X2 = X2 + xc - hub_x_distance;
Y2 = Y2 + yc - hub_y_distance;
Z2 = Z2*0.4 + zc*0.99 - hub_z_distance*2;
surf(X2,Y2,Z2)


[X3,Y3,Z3] = cylinder(rotor_diameter/2, 30);
X3 = X3 + xc + hub_x_distance;
Y3 = Y3 + yc - hub_y_distance;
Z3 = Z3*0.4 + zc*0.99 - hub_z_distance*2;
surf(X3,Y3,Z3)

[X4,Y4,Z4] = cylinder(rotor_diameter/2, 30);
X4 = X4 + xc + hub_x_distance;
Y4 = Y4 + yc + hub_y_distance;
Z4 = Z4*0.4 + zc*0.99 - hub_z_distance*2;
surf(X4,Y4,Z4)

[XT1,YT1,ZT1] = cylinder(rotor_diameter/9, 30);
XT1 = XT1 + xc - hub_x_distance;
YT1 = YT1 + yc + hub_y_distance;
ZT1 = ZT1*0.4 + zc*0.99 - hub_z_distance*2;
surf(XT1,YT1,ZT1)

[XT2,YT2,ZT2] = cylinder(rotor_diameter/9, 30);
XT2 = XT2 + xc - hub_x_distance;
YT2 = YT2 + yc - hub_y_distance;
ZT2 = ZT2*0.4 + zc*0.99 - hub_z_distance*2;
surf(XT2,YT2,ZT2)

[XT3,YT3,ZT3] = cylinder(rotor_diameter/9, 30);
XT3 = XT3 + xc + hub_x_distance;
YT3 = YT3 + yc - hub_y_distance;
ZT3 = ZT3*0.4 + zc*0.99 - hub_z_distance*2;
surf(XT3,YT3,ZT3)

[XT4,YT4,ZT4] = cylinder(rotor_diameter/9, 30);
XT4 = XT4 + xc + hub_x_distance;
YT4 = YT4 + yc + hub_y_distance;
ZT4 = ZT4*0.4 + zc*0.99 - hub_z_distance*2;
surf(XT4,YT4,ZT4)

%  [xt yt zt] = cylinder(0.1, 7);
%  surf(xt + xc*0.9, yt + yc*0.9 + 5, zt , 'EdgeColor', 'none', 'FaceColor', 'm');



% h1 = prism(-5, -0.25, -0.25, 10, 0.5, 0.5);
% h2 = prism(-0.25, -5, -0.25, 0.5, 10, 0.5);
% surf(h1)
% surf(h2)
% Copyright - Carlos Montalvo 2015
% You may freely distribute this file but please keep my name in here
% as the original owner
