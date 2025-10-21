%%%Plot Actuators

parameters

MiscData = dlmread([root1,'Controls.OUT']);
%write(91,fmt='(1000F30.10)')
%T%SIM%TIME,T%PAP%TLBAR,T%PAP%TRBAR,T%THR%THETAREEL,T%THR%THETADOTREEL,T%THR%TORQUE,T%THR%TORQUECOMMAND,T%THR%THETAINT,thetacommand,ldotcommand,ldotnominal,stretchlen,len,incidence

nombrake = 0.4; %%0-1
maxbrake = 63; %%cm
nomincidence = 0.0610;

time = MiscData(:,1);
left = maxbrake*(MiscData(:,2));
right = maxbrake*(MiscData(:,3));
THETAREEL = MiscData(:,4);
THETADOTREEL = MiscData(:,5);
TORQUE = MiscData(:,6);
Qcommand = MiscData(:,7);
thetaint = MiscData(:,8);
thetacommand = MiscData(:,9);
ldotcommand = MiscData(:,10);
ldotnominal = MiscData(:,11);
stretchlen = MiscData(:,12);
nomlen = MiscData(:,13);
incidence = MiscData(:,14);

if sum(left-right) == 0
    plottool(1,'Controls',18,'Time(sec)','Symmetric Brake Deflection(cm)');
    plot(time,left,'k-','LineWidth',2)
    plot(time,0*time+63*nombrake,'k--','LineWidth',2)
    legend('Controlled','Uncontrolled')
else
    plottool(1,'Controls',18,'Time(sec)','Brake Deflection(cm)');
    plot(time,left,'k-','LineWidth',2)
    plot(time,right,'k--','LineWidth',2)
    plot(time,0*time+63*nombrake,'k-.','LineWidth',2)
    legend('Left Brake','Right Brake','Nominal Symmetric Brake')
end
plot(time,0*time,'r-','LineWidth',2)
plot(time,0*time+63,'r-','LineWidth',2)

ylim([-10 70])

plottool(1,'Theta Reel',18,'Time(sec)','Theta Reel(rad)');
plot(time,THETAREEL,'k-','LineWidth',2)
plot(time,thetacommand,'r-','LineWidth',2)
legend('Theta(rad)','Thetacommand(rad)')

plottool(1,'ThetaDot Reel',18,'Time(sec)','ThetaDot Reel(rad/s)');
plot(time,THETADOTREEL,'k-','LineWidth',2)

plottool(1,'Torque',18,'Time(sec)','Torque(N-m)');
plot(time,TORQUE,'k-','LineWidth',2)
plot(time,Qcommand,'r-','LineWidth',2)
legend('Torque','Torque Command')

plottool(1,'Theta Int',18,'Time(sec)','Theta Int');
plot(time,thetaint,'k-','LineWidth',2)

plottool(1,'Ldotcommand',18,'Time(sec)','Pay Out Rate (m/s)');
plot(time,ldotcommand,'k-','LineWidth',2)
%plot(time,ldotnominal,'r-','LineWidth',2)
%legend('Command','Nominal')

plottool(1,'StretchedLength',18,'Time(sec)','Stretch Length (m)');
plot(time,stretchlen,'k-','LineWidth',2)
plot(time,nomlen,'k--','LineWidth',2)

plottool(1,'Percentage of Stretch',18,'Time (sec)','Stretch Percentage (%)');
plot(time,100*(stretchlen-nomlen)./nomlen,'k-','LineWidth',2)

plottool(1,'Incidence Angle',18,'Time (sec)','Incidence Angle (deg)');
plot(time,180/pi*(incidence+nomincidence),'k-','LineWidth',2)
plot(time,0*time+180/pi*nomincidence,'k--','LineWidth',2)
