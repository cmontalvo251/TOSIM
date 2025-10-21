%plotCEPs
purge
files = {'No_Winds_Heli_Motion.OUT','Winds_No_Heli_Motion.OUT','Winds_Heli_Motion.OUT'};

[h1,ax1] = plottool(1,'Heli Frame',18,'Delta X (ft)','Crossrange (ft)');
[h2,ax2] = plottool(1,'Normalized Frame',18,'Biased X (ft)','Biased Y (ft)');

colors = {'b','r','g'};
p = [];
for idx = 1:length(files)
    tZ = dlmread(['Output_Files/',files{idx}]);
    t = tZ(:,1);
    Z = tZ(:,2:end);
    %%%Compute Tether angle
    Zheli = Z(:,14:25);
    Ztowed = Z(:,1:13);
    time = t;
    xtowed = Ztowed(:,1);
    ytowed = Ztowed(:,2);
    reelx = Zheli(:,1);
    deltax1 = reelx(1) - xtowed(1);
    deltax = (reelx-xtowed);
    deltay = ytowed(1)-ytowed;
    xmean = mean(deltax-deltax1);
    ymean = mean(deltay);
    pnew = plot(ax1,deltax-deltax1,deltay,[colors{idx},'-'],'LineWidth',2);
    plot(ax2,deltax-deltax1-xmean,deltay-ymean,[colors{idx},'-'],'LineWidth',2);
    p = [p;pnew];
    
    %%%Plot a CEP Curve
    radius = sort(sqrt((deltax-deltax1-xmean).^2+(deltay-ymean).^2));
    CEP = radius(round(length(radius)/2));
    disp('Mean X,Y (ft) and CEP')
    disp([xmean ymean CEP]')
    theta = linspace(0,2*pi,100);
    xcircle = CEP*cos(theta)+xmean;
    ycircle = CEP*sin(theta)+ymean;
    plot(ax1,xcircle,ycircle,[colors{idx},'--'],'LineWidth',2)
    plot(ax2,xcircle-xmean,ycircle-ymean,[colors{idx},'--'],'LineWidth',2)
end
axis(ax1,'equal')
axis(ax2,'equal')
legend(ax2,p,'Winds=0 ft/s Helicopter=Turbulent','Winds=15 ft/s Helicopter=Steady','Winds=15 ft/s Helicopter = Turbulent')
legend(ax1,p,'Winds=0 ft/s Helicopter=Turbulent','Winds=15 ft/s Helicopter=Steady','Winds=15 ft/s Helicopter = Turbulent')