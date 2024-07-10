purge

close all

parameters

%%%%%%%%%%TOWED PARAMS%%%%%%%%%%

VAR_PARAM1 = 'TOWED WLTETHER';
VAR_PARAM2 = 'TOWED SLTETHER';

[xh,yh,zh]=stlread('helicopter.stl');
xyzh = [xh;yh;zh];

[h2,f2] = plottool(1,'Visualization',14,'x','y','z');

FILE = 'Input_Files/Tethers/TOMAD.THR';
ROW1 = 25;
INLINE1 = '!Waterline of Tether Attachment Point on Towed in Towed Reference Frame(ft)';
varin1 = linspace(-0.5,0,4);
nom1 = -0.25; %%%Nominal is actually -0.1979 but I haven't simulated this yet so for now nominal is -0.25 REVISIT REVISIT
[h1,f1] = plottool(1,'Theta',18,'Time (sec)','\phi (deg)');
savetime = 50;
xaxis = 'Waterline Connection Point (ft)';

ROW2 = 23;
INLINE2 = '!Stationline of Tether Attachment Point on Towed in Towed Reference Frame(ft)';
varin2 = linspace(-0.5,0.5,4);
nom2 = 0.0;
yaxis = 'Stationline Connection Point (ft)';

addpath 'plotting/'

system('make rebuild');
system('rm Output_Files/*.OUT');
system('rm Output_Files/*.txt');
system(['cp ',FILE,' ',FILE,'.backup']);

data = {};
fid = fopen(FILE);
THISLINE = fgetl(fid);
while THISLINE ~= -1;
   data = [data;THISLINE];
   THISLINE = fgetl(fid);
end

STDFILE = 'Output_Files/STD.OUT';
MEANFILE = 'Output_Files/MEAN.OUT';
stdfid = fopen(STDFILE);
meanfid = fopen(MEANFILE);
color = {'b-','r-','g-','m-','k-','y-','c-'};

time = zeros(length(varin1),length(varin2));
magptp = zeros(length(varin1),length(varin2));
ctr1 = 0;
cctr = 0;
outctr = 0;
for var1 = varin1
    ctr1 = ctr1 + 1;
    ctr2 = 0;
    for var2 = varin2
    
        ctr2 = ctr2 + 1;
        cctr = cctr + 1;
        outctr = outctr + 1;
        if cctr > length(color)
            cctr = 1;
        end
    
        OUTLINE1 = [num2str(var1),' ',INLINE1];
        OUTLINE2 = [num2str(var2),' ',INLINE2];
    
        disp(OUTLINE1)
        disp(OUTLINE2)
    
        data{ROW1} = OUTLINE1;
        data{ROW2} = OUTLINE2;
    
        writedata(FILE,data);
        tic
        system('./LinuxRun.exe Input_Files/TOMAD.ifiles &> file');
        %systemvar1 = var1in('./LinuxRun.exe Input_Files/TOMAD.ifiles');
        time(ctr1,ctr2) = toc
        system('rm file');
        system(['cp Output_Files/State.OUT Output_Files/State',num2str(var1),num2str(var2),'.OUT']);

        tZ = dlmread('Output_Files/State.OUT');
        t = tZ(:,1);
        Z = tZ(:,2:end);
        ptp = quat2euler(Z(:,5:7),0,t,'Towed');
        mag = sqrt(sum(ptp(:,1).^2)  + sum(ptp(:,2).^2) + sum(ptp(:,3).^2));
        
        magptp(ctr1,ctr2) = mag;

        plot(f1,t,ptp(:,1)*180/pi,color{cctr},'LineWidth',2)
        drawnow


        

        % magptpmesh = ones(3,3)*sum(magptp);

        % surf(f1,V1mesh,V2mesh,magptpmesh)
        % hold on
        set(0,'currentfigure',h2)
        plotSteadyState(color{cctr},xyzh);
    end
end

%%%%Reset file to nominal
OUTLINE1 = [num2str(nom1),' ',INLINE1];
OUTLINE2 = [num2str(nom2),' ',INLINE2];
disp(OUTLINE1)
disp(OUTLINE2)
data{ROW1} = OUTLINE1;
data{ROW2} = OUTLINE2;
writedata(FILE,data);

light('Position',-1000.*[0 0 2],'Style','infinite');

% Dafuq is this
Sensitivity2D(varin1,varin2,savetime,xaxis,yaxis)

% 3D Plot that Montalvo wants
figure()
[SL,WL] = meshgrid(varin2,varin1)
surf(SL,WL,magptp)