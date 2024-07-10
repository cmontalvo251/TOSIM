%%%%All of this data has been pulled from John Nicolaides work from 1971
purge

AR = [0.83 0.78 0.5 0.64 0.94 1.0 1.5 1.77 2.0 2.5 3.0];
CL0 = [0.25 0.25 0.18 0.25 0.3 0.3 0.36 0.2 0.36 0.36 0.5];
%%%%Drag Plot is unreadale so I'm setting them all to be the same
CD0 = 0.2*ones(1,length(AR));
%%%CD at aoa = 25 degrees
CD25 = 0.5;
aoa25 = 25*pi/180;
CDA2 = (0.5-CD0(1))/(aoa25.^2)*ones(1,length(AR));

%%%The data below has been shifted so do not use it in any calculations
%%%except in calculating CLA
%%%CL at 5 degrees 
CL5 = [0.42 0.3 0.27 0.39 0.25 0.42 0.48 0.39 0.5 0.58 0.65];
%%%CL at -5 degrees
CLm5 = zeros(1,length(AR));

%%%Test Plot
plottool(1,'CL Shifted',18,'Angle of Attack (Deg)','CL');
plot([-5 5],[CLm5;CL5],'LineWidth',2)
legend(vec2cell(AR))

%%%Compute CLA
CLA = (CL5-CLm5)./(10*pi/180);

%%%Sort by Aspect Ratio
[AR,I] = sort(AR);
CL0 = CL0(I);
CLA = CLA(I);
CD0 = CD0(I);
CDA2 = CDA2(I);

%%%%Fit CLA and CD2
p = polyfit(AR,CLA,1);
AR_fit = linspace(0.5,5,100);
cla_fit = polyval(p,AR_fit);

plottool(1,'CL0',18,'Aspect Ratio','CL0');
plot(AR,CL0)
plottool(1,'CLA',18,'Aspect Ratio','CLA');
plot(AR,CLA)
plot(AR_fit,cla_fit,'--')
plottool(1,'CD0',18,'Aspect Ratio','CD0');
plot(AR,CD0)
plottool(1,'CDA',18,'Aspect Ratio','CDA2');
plot(AR,CDA2)

%%%Test Plots
AOA = linspace(-10,10,100).*pi/180;
CL = CL0'*ones(1,length(AOA)) + CLA'*AOA;
CD = CD0'*ones(1,length(AOA)) + CDA2'*AOA.^2;
LD = CL./CD;

plottool(1,'CL',18,'Angle of Attack (Deg)','CL');
plot(AOA*180/pi,CL,'LineWidth',2)
legend(vec2cell(AR))
ylim([0 1.2])

plottool(1,'CD',18,'Angle of Attack (Deg)','CD');
plot(AOA*180/pi,CD,'LineWidth',2)
legend(vec2cell(AR))

plottool(1,'L/D',18,'Angle of Attack (Deg)','L/D');
plot(AOA*180/pi,LD,'LineWidth',2)
legend(vec2cell(AR))

