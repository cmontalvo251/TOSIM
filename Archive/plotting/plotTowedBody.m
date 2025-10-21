%%%%Actual data from SimX_Aircraft/TowBody2.wings file
clear
clc
close all
r = [1.1592 2.509 3.368 4.286 4.750 4.750 4.750 4.750 4.750]/24; %%%%Just fill this out and the code will work
x = [0.172 0.86 1.72 3.440 5.160 6.235 11.722 16.340 40.]/12;
xcg = 15/12;
rcg = 0;

plot(x,r)
hold on
plot(xcg,rcg,'ks','MarkerSize',20)
plot(x,-r)
xlabel('Station Line (feet)')
ylabel('Radius (feet)')
axis equal
%%%%Interpolating at equally spaced points
xinterp = linspace(x(1),x(end),length(x));
rinterp = interp1(x,r,xinterp);

%%%Creating a mesh
[Xg,Yg,Zg] = cylinder(rinterp);
Zg = Zg*x(end);
%%%Plotting the cylinder
figure()
mesh(Xg,Yg,Zg)
xlabel('Butt line (feet)')
ylabel('Water line (feet)')
zlabel('Station line (feet)')
axis equal