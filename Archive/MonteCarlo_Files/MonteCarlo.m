purge

close all

parameters

%%%ENVIRONMENT

%VAR_PARAM = 'WINDS';
%VAR_PARAM = 'CONSTANT';
%VAR_PARAM = 'HELI VELOCITY';
%VAR_PARAM = 'ROTORCRAFT NOISE';
%VAR_PARAM  = 'WIND FREQUENCY';
VAR_PARAM = 'HELI FREQUENCY';

%%%%%%%%%%TOWED PARAMS%%%%%%%%%%

%VAR_PARAM = 'TOWED WEIGHT'; %%% NEED TO CHANGE INERTIA
%VAR_PARAM = 'TOWED WLTETHER';
%VAR_PARAM = 'TOWED SLTETHER';
%VAR_PARAM = 'TOWED SIMTABLES';

%%%%%%%TETHER PARAMS%%%%%%%%%%%%%

%VAR_PARAM = 'TETHER MOMENT';
%VAR_PARAM = 'TETHER DIAMETER';
%VAR_PARAM = 'TETHER WEIGHT';
%VAR_PARAM = 'TETHER KE';
%VAR_PARAM = 'TETHER KV';
%VAR_PARAM = 'TETHER CV';
%VAR_PARAM = 'TETHER LENGTH';

[xh,yh,zh]=stlread('helicopter.stl');
xyzh = [xh;yh;zh];

%[h2,f2] = plottool(1,'Visualization',14,'x','y','z');

switch VAR_PARAM
    case 'WIND FREQUENCY'
        FILE = 'Input_Files/TOMAD.ATM';
        ROW = 6;
        INLINE = 'Period (sec)';
        nom = 0;
        savetime = 50;
        xaxis = 'Period of Atmospheric Disturbance';
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
        varin = linspace(1,20,10);
    case 'HELI FREQUENCY'
        FILE = 'Input_Files/Helis/Steady.HELI';
        ROW = 21;
        INLINE = 'Period (sec)';
        nom = 0;
        savetime = 50;
        xaxis = 'Period of Helicopter Disturbance (sec)';
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
        varin = linspace(1,20,100);
    case 'ROTORCRAFT NOISE'
        FILE = 'Input_Files/Helis/Steady.HELI';
        ROW = 19;
        INLINE = 'Rotorcraft-Noise (ft/s^2)';
        nom = 0.0;
        savetime = 50;
        xaxis = 'Standard Deviation of Helicopter Velocity (ft/s)';
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
        varin = linspace(0,1000,10);
    case 'HELI VELOCITY'
        FILE = 'Input_Files/Helis/Steady.HELI';
        ROW = 14;
        INLINE = ' !Heli_speed(ft/s) (101.60721)';
        nom = 101.60721;
        varin = linspace(10,2*nom,10)
        savetime = 100;
        xaxis = 'Velocity of Helicopter (ft/s)';
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
    case 'TOWED SIMTABLES';
        FILE = 'Input_Files/TOMAD.ifiles';
        ROW = 6;
        INLINE = ' !Towed WingsX input file directory';
        nom = 'Baseline/';
        varin = {'001 - Tail only','002 - Tail double span','003 - Stretched by 1.5 longitudinally','004 - CG @ 25% from nose','005 - CG 15%','006 - Tail 1.5 span','007 - CG @ 20% from nose','008 - Tail 1.75x span','009 - Tail 2x chord','010 - Drag Skirt','011 - Drag skirt 0.5x diameter','012 - Neg Wing Incidence 10 deg','013 - Neg Wing Dihedral 30deg','014 - Flaperon Baseline','Baseline/'};
        savetime = 50;
        xaxis = 'Configuration Number';
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
    case 'TETHER LENGTH'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 7;
        INLINE = '!Unstretched total length of tether (ft)';
        nom = 196;
        varin = linspace(0.5*nom,nom*1.5,5); 
        savetime = 50;
        xaxis = 'Unstretched Total Length of Tether (ft)';
    case 'TETHER KE'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 11;
        INLINE = '!(Ke) Modulus of Tether line (lbf/ft^2)';
        nom = 36431800;
        varin = linspace(0.1*nom,nom*1.2,5); 
        savetime = 50;
        xaxis = '(Ke) Modulus of Tether line (lbf/ft^2)';
    case 'TETHER KV'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 12;
        INLINE = '!(Kv) Modulus of Tether line (lbf/ft^2)';
        nom = 60000000;
        varin = linspace(nom*0.1,nom*1.2,5); 
        savetime = 50;
        xaxis = '(Kv) Modulus of Tether line (lbf/ft^2)';
    case 'TETHER CV'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 13;
        INLINE = '!(Cv) Damping Modulus of Tether line (lbf-s/ft^2)';
        nom = 1111112;
        varin = linspace(0.8*nom,nom*1.2,5); 
        savetime = 50;
        xaxis = INLINE;
    case 'TETHER WEIGHT'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 6;
        INLINE = '!Mass per unit length (slugs/ft)';
        nom = 5.668e-5;
        varin = linspace(nom*0.5,nom*2,5); 
        savetime = 50;
        xaxis = 'Mass per Unit Length (slugs/ft)';
    case 'TETHER DIAMETER'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 10;
        INLINE = '!Diameter of tether line (ft) (Guess)';
        varin = linspace(0.02208*0.5,0.02208*2.0,5); 
        nom = 0.02208;
        savetime = 50;
        xaxis = 'Diameter of tether line (ft)';
    case 'TOWED WEIGHT'
        FILE = 'Input_Files/TOWED/TowBody2.coef';
        ROW = 10;
        INLINE = 'Reference level flight lift (lbf) (8.0)';
        varin = [2 4 6 8];
        savetime = 50;
        xaxis = 'Towed Body Weight (lbf)';
        nom = 2;
    case 'TETHER MOMENT'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 16;
        INLINE = '! Non-linearity of Tether (-1=off,0=linear, 1 = non-linear)';
        varin = [-1 0 1];
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
    %case 'TETHER LENGTH'
    %    [h2,f2] = plottool(1,'Tether Length',18,'Time(sec)','Change in Tether Length (ft)');
    case 'TOWED WLTETHER'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 25;
        INLINE = '!Waterline of Tether Attachment Point on Towed in Towed Reference Frame(ft)';
        varin = linspace(-0.25,0,5);
        nom = -0.25; %%%Nominal is actually -0.1979 but I haven't
                     %simulated this yet so for now nominal is
                     %-0.25 REVISIT REVISIT
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
        savetime = 50;
        xaxis = 'Waterline Connection Point (ft)';
    case 'TOWED SLTETHER'
        FILE = 'Input_Files/Tethers/TOMAD.THR';
        ROW = 23;
        INLINE = '!Stationline of Tether Attachment Point on Towed in Towed Reference Frame(ft)';
        varin = linspace(0,18,5)/12; %feet
        nom = 0.0;
        savetime = 50;
        xaxis = 'Stationline Connection Point (ft)';
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
    case 'WINDS'
        FILE = 'Input_Files/WRF.ATM';
        %ROW = 6;
        %INLINE = '!Vy_Wave_speed(ft/s)';
        %varin = linspace(-9.02,9.02,5);
        %xaxis = 'Random Seed';
        %nom = -9.02;
        
        ROW = 3;
        INLINE = '!Wind scale';
        varin = linspace(0,1,5);
        xaxis = 'Mean Atmospheric WindSpeed (ft/s)';
        nom = 1;

        savetime = 50;
        [h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
    case 'CONSTANT'
        FILE = '';
        ROW = [];
        INLINE = '';
        varin = 1:5;
        nom = [];
        savetime = 50;
        xaxis = 'Simulation Number';
        [hw,fw] = plottool(1,'Winds',18,'Time (sec)','Winds (m/s)');
        [hp,fp] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
    otherwise
        disp('Invalid VAR_PARAM')
        return
end

%FILE = 'Input_Files/TOMAD.ATM';
%FILE = 'Input_Files/Tethers/TOMAD.THR';
%INLINE = '!Number of beads(May not exceed 100 unless you edit the source code)';
%INLINE = '!Mass per unit length (slugs/ft)';
%INLINE = '!Windspeed (ft/s) (5.38)';
%INLINE = '!Diameter of tether line(ft) (Guess)';
%INLINE = 'Unstretched total length of tether (ft)';
%INLINE = '!Stationline of Tether Attachment Point on Towed in Towed Reference Frame(ft)';
%INLINE = '!Normal Drag(nd) (1.0)';
%varin = [30:-5:5];
%varin = [0:2:18]./12;
%varin = [0.019872 0.02208 0.024288];
%varin = [176.4 196 215.6];
%varin = [0.5 1.0];
%varin = [2*5.668E-05 5.668E-05];

addpath 'plotting/'

system('make');
system('rm Output_Files/*.OUT');
system('rm Output_Files/*.txt');
if length(FILE)
    system(['cp ',FILE,' ',FILE,'.backup']);
    data = {};
    fid = fopen(FILE);
    THISLINE = fgetl(fid);
    while THISLINE ~= -1;
        data = [data;THISLINE];
        THISLINE = fgetl(fid);
    end
end

STDFILE = 'Output_Files/STD.OUT';
MEANFILE = 'Output_Files/MEAN.OUT';
stdfid = fopen(STDFILE);
meanfid = fopen(MEANFILE);
color = {'b-','r-','g-','m-','k-','y-','c-'};

time = zeros(1,length(varin));
ctr = 0;
outctr = 0;
for var = varin
    
    ctr = ctr + 1;
    outctr = outctr + 1;
    if ctr > length(color)
        ctr = 1;
    end
    
    if length(FILE)
        if iscell(varin)
            OUTLINE = ['''WINGSDIR'' ''SimX_Aircraft_Data/Towbody/',var{1},'/''',INLINE];
        else
            OUTLINE = [num2str(var),' ',INLINE];
        end
    
        disp(OUTLINE)
        
        data{ROW} = OUTLINE;
    
        writedata(FILE,data);
    end
    
    tic
    system('./LinuxRun.exe Input_Files/TOMAD.ifiles &> file');
    %system('./LinuxRun.exe Input_Files/TOMAD.ifiles');
    time(ctr) = toc
    system('rm file');
    if iscell(varin)
        system(['cp Output_Files/State.OUT Output_Files/State',num2str(outctr),'.OUT']);
    else
        system(['cp Output_Files/State.OUT Output_Files/State',num2str(var),'.OUT']);
        system(['cp Output_Files/Misc.OUT Output_Files/Misc',num2str(var),'.OUT']);
    end
    
    switch VAR_PARAM
        %case 'TETHER LENGTH'
        %    plotTetherLength(color{ctr},var);
        case 'TETHER MOMENT'
            ctr = 2;
            tZ = dlmread('Output_Files/State.OUT');
            t = tZ(:,1);
            Z = tZ(:,2:end);
            ptp = quat2euler(Z(:,4:7),0,t,'Towed');
            plot(t,ptp(:,1)*180/pi,color{ctr-1},'LineWidth',2)
            plot(t,ptp(:,2)*180/pi,color{ctr},'LineWidth',2)
            plot(t,ptp(:,3)*180/pi,color{ctr+1},'LineWidth',2)
        case 'TOWED WLTETHER'
            tZ = dlmread('Output_Files/State.OUT');
            t = tZ(:,1);
            Z = tZ(:,2:end);
            ptp = quat2euler(Z(:,4:7),0,t,'Towed');
            plot(f1,t,ptp(:,1)*180/pi,color{ctr},'LineWidth',2)
            drawnow
            set(0,'currentfigure',h2)
            plotSteadyState(color{ctr},xyzh);
        case 'CONSTANT'
            winds = dlmread('Output_Files/Misc.OUT');
            timew = winds(:,1);
            plot(fw,timew,winds(:,2:4),color{ctr},'LineWidth',2)
            drawnow
            tZ = dlmread('Output_Files/State.OUT');
            t = tZ(:,1);
            Z = tZ(:,2:end);
            ptp = quat2euler(Z(:,4:7),0,t,'Towed');
            plot(fp,t,ptp(:,1)*180/pi,color{ctr},'LineWidth',2)
            drawnow
        otherwise
            tZ = dlmread('Output_Files/State.OUT');
            t = tZ(:,1);
            Z = tZ(:,2:end);
            ptp = quat2euler(Z(:,4:7),0,t,'Towed');
            plot(f1,t,ptp(:,1)*180/pi,color{ctr},'LineWidth',2)
            drawnow
            if exist('h2','var')
                set(0,'currentfigure',h2)
                plotSteadyState(color{ctr},xyzh);
            end
    end
end

%%%%Reset file to nominal
if length(FILE)
    if iscell(varin)
        OUTLINE = ['''WINGSDIR'' ''SimX_Aircraft_Data/Towbody/',nom,'''',INLINE];
    else
        OUTLINE = [num2str(nom),' ',INLINE];
    end
    disp(OUTLINE)
    data{ROW} = OUTLINE;
    writedata(FILE,data);

    if iscell(varin)
        varin = 1:length(varin);
    end
end

switch VAR_PARAM
    case 'TETHER MOMENT'
        legend('Phi','Theta','Psi')
    otherwise
        legend(vec2cell(varin))
        light('Position',-1000.*[0 0 2],'Style','infinite');
end

Sensitivity(varin,savetime,xaxis)
%SteadyState(varin)
%ProcessMonteCarlo
