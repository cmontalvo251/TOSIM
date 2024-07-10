%%%%Plot Sweeps
close all
clc
data = dlmread('Output_Files/SWEEPS_Tether.OUT');
        
aoa = data(:,1);
ub = data(:,2);
wb = data(:,3);
FXAERO = data(:,4);
FYAERO = data(:,5);
FZAERO = data(:,6);

plottool(1,'FXAERO',18,'AOA (deg)','X Body Aerodynamic Force (lbf)');
plot(aoa*180/pi,FXAERO,'LineWidth',2)

plottool(1,'FZAERO',18,'AOA (deg)','Z Body Aerodynamic Force (lbf)');
plot(aoa*180/pi,FZAERO,'LineWidth',2)
