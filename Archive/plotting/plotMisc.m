%%%Plot Actuators

MiscData = dlmread('Output_Files\Controls.OUT');
%write(91,fmt='(1000F30.10)') T%SIM%TIME,T%PAP%TLBAR,T%PAP%TRBAR,T%THR%THETAREEL,T%THR%THETADOTREEL,T%THR%TORQUE

time = MiscData(:,1);
left = 100*MiscData(:,2);
right = 100*MiscData(:,3);
THETAREEL = MiscData(:,4);
THETADOTREEL = MiscData(:,5);
TORQUE = MiscData(:,6);

plottool(1,'Controls',18,'Time(sec)','Brake Deflection(%)');
plot(time,left,'k-','LineWidth',2)
plot(time,right,'k--','LineWidth',2)

plottool(1,'Theta Reel',18,'Time(sec)','Theta Reel(rad)');
plot(time,THETAREEL,'k-','LineWidth',2)

plottool(1,'Theta Reel',18,'Time(sec)','ThetaDot Reel(rad/s)');
plot(time,THETADOTREEL,'k-','LineWidth',2)

plottool(1,'Theta Reel',18,'Time(sec)','Torque(N-m)');
plot(time,TORQUE,'k-','LineWidth',2)
