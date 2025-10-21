purge

files = {'Output_Files/Sweeps/SWEEPS_Taylor.OUT','Output_Files/Sweeps/SWEEPS_WingsX.OUT'};

[h1,f1] =  plottool(1,'CL',18,'AOA (deg)','CL');
[h2,f2] = plottool(1,'CD',18,'AOA (deg)','CD');
[h3,f3] = plottool(1,'CM',18,'AOA (deg)','CM');

for idx = 1:length(files)
    file = files{idx};
    data = dlmread(file);
    aoa = data(:,1);
    ub = data(:,2);
    wb = data(:,3);
    CL = data(:,4);
    CD = data(:,5);
    CM = data(:,10);
    plot(f1,aoa*180/pi,CL,'LineWidth',2)
    plot(f2,aoa*180/pi,CD,'LineWidth',2)
    plot(f3,aoa*180/pi,CM,'LineWidth',2)
end