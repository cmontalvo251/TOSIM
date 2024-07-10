%%%%Plot Sweeps
clc
try
    data = dlmread('Output_Files/SWEEPS_Taylor.OUT');
catch me
    try
        data = dlmread('Output_Files/SWEEPS_WingsX.OUT');
    catch me
        return;
    end
end
        
aoa = data(:,1);
ub = data(:,2);
wb = data(:,3);
CL = data(:,4);
CD = data(:,5);
Cx = data(:,6);
Cy = data(:,7);
Cz = data(:,8);
Cl = data(:,9);
Cm = data(:,10);
Cn = data(:,11);

plottool(1,'CL',18,'AOA (deg)','CL');
plot(aoa*180/pi,CL,'LineWidth',2)

plottool(1,'CD',18,'AOA (deg)','CD');
plot(aoa*180/pi,CD,'LineWidth',2)

plottool(1,'CM',18,'AOA (deg)','Cm');
plot(aoa*180/pi,Cm,'LineWidth',2)
