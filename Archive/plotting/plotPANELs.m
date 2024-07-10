purge

%fid = fopen('Input_Files/Parafoils/Parafoil107_CradleNew_Panels.PAPA','r');
%fid = fopen('Input_Files/Parafoils/Airdrop_Canopy_Panels.PAPA','r');
fid = fopen('Input_Files/Parafoils/SpirulineL.PAPA','r');

for ii = 1:9
    a = fgetl(fid);
end
str = fgetl(fid);
dx = str2num(str(1:find(str=='!')-1));
for ii = 1:16
    a = fgetl(fid);
end
rGC = [0;0;0];
str = fgetl(fid);
rGC(1) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGC(2) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGC(3) = str2num(str(1:find(str=='!')-1));

rGP = [0;0;0];
str = fgetl(fid);
rGP(1) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGP(2) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGP(3) = str2num(str(1:find(str=='!')-1))

for ii = 1:27
    a = fgetl(fid);
end

npanels_str = fgetl(fid);
npanels = str2num(npanels_str(1:find(npanels_str=='!')-1))

%%%Read in Area
ELES = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELES(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Stationline
ELESL = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELESL(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Buttline
ELEBL = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELEBL(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Waterline
ELEWL = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELEWL(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Phi
ELEPHI = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELEPHI(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Theta
ELETHETA = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELETHETA(ii) = str2num(str(1:find(str=='!')-1));
end

%%%%plot panels and compute the aerodynamic center
plottool(1,'Panels',12);
xaero = 0;yaero = 0;zaero = 0;
ELEWL=[         -0.3
         -0.3
   -1.0536
   -1.0536
   -1.5456
   -1.5456
   -1.6818
         0.5
         0.5];
ELEBL=[       -4.1000
    4.1000
   -2.9333
    2.9333
   -1.4667
    1.4667
         0
   -3.3000
    3.3000];
for ii = 1:npanels-2
    dy = ELES(ii)/dx;
    dz = 0.1;
    xc = ELESL(ii) + rGP(1);
    yc = 0.8*ELEBL(ii) + rGP(2);
    %yc = ELEBL(ii) + rGP(2);
    zc = ELEWL(ii) + rGP(3);
    phi = ELEPHI(ii);
    theta = ELETHETA(ii);
    psi = 0;
    CubeDraw(dx,dy,dz,xc,yc,zc,phi,theta,psi,[1 0 0]);
    
    xaero = xaero + ELES(ii)*(dx/4 + ELESL(ii) + rGP(1))/sum(ELES);
    yaero = yaero + ELES(ii)*(ELEBL(ii) + rGP(2))/sum(ELES);
    zaero = zaero + ELES(ii)*(ELEWL(ii) + rGP(3))/sum(ELES);
end

%%%Plot parafoil aerocenter
rGAp = [xaero;yaero;zaero]
rPAp = rGP-rGAp
%ball(rGAp(1),rGAp(2),rGAp(3),0.5)

%%%%Plot gimbal joint
%ball(0,0,0,0.8)

%%%plot parafoil cg
%ball(rGP(1),rGP(2),rGP(3),0.5)

%%%Plot cradle
CubeDraw(2,2,2,rGC(1),rGC(2),rGC(3),0,0,0,'none')

reverse(['y','z'])

view([90 0])

axis equal
axis equal

fclose all;