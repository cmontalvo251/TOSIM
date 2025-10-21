function rF = DrawParafoil(xg,yg,zg,phip,thetap,psip,phic,thetac,psic)

dx = 2.1;
rGC = [0;0;0.47];
rGP = [0;0;-5.75];
npanels = 9;

%%%Area
ELES = [2.6429
    2.6429
    2.6429
    2.6429
    2.6429
    2.6429
    2.6429
    0.9250
    0.9250];
%%%Stationline
ELESL = zeros(npanels,1);
%%%Phi
ELEPHI =  [  -0.7854
    0.7854
   -0.5236
    0.5236
   -0.2618
    0.2618
         0
    1.5708
   -1.5708];
%%%Theta
ELETHETA = zeros(npanels,1);

%%%%plot panels and compute the aerodynamic center
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

%%%Plot cradle
TIC = R123(phic,thetac,psic);
rG = [xg;yg;zg];
rC = rG + TIC*rGC;
rF = rC + TIC*[0.3;0;0.23];
CubeDraw(0.6,1,0.68,rC(1),rC(2),rC(3),phic,thetac,psic,'none')
ball(rF(1),rF(2),rF(3),0.2)
plot3([rC(1) rG(1)],[rC(2) rG(2)],[rC(3) rG(3)],'k-','LineWidth',1)
TIP = R123(phip,thetap,psip);
for ii = 1:npanels-2
    dy = ELES(ii)/dx;
    dz = 0.1;
    %xc = ELESL(ii) + rGP(1);
    %yc = 0.8*ELEBL(ii) + rGP(2);
    %yc = ELEBL(ii) + rGP(2);
    %zc = ELEWL(ii) + rGP(3);
    rP = rG + TIP*(rGP+[ELESL(ii);0.8*ELEBL(ii);ELEWL(ii)]);
    plot3([rP(1) rG(1)],[rP(2) rG(2)],[rP(3) rG(3)],'k-','LineWidth',1)
    phi = ELEPHI(ii)+phip;
    theta = ELETHETA(ii)+thetap;
    psi = 0+psip;
    CubeDraw(dx,dy,dz,rP(1),rP(2),rP(3),phi,theta,psi,[1 0 0]);
end
