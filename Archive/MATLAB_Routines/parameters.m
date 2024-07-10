%%%%%This file should be the only file you need to change and do stuff to
COMPILECODE = 1;
RUNCODE = 1;
COMPARE = 0;
OFFSET = 0;%%Seconds (Only used during COMPARE)
SAVEMOVIE = 0;
STLPLOT = 0;
saveplots = 0;
PLOTQUADDOTSTATES = 0;

pheli = 0;
ptowed = 1;
ptether = 0;
pcontrols = 0;
pwind = 0;
pforces = 0;
perrors = 0;
psweeps = 0;

vheli = 1;
vtowed = 1;
vtether = 1;
skip = 10;

%Az = -44;
%El = 20;

%%%Isometric
Az = 65; 
El = 16;
%%%%XZ plane - Side View
%Az = 0;
%El = 0;
%%%%XY Plane - Top View
%Az = 0;
%El = 90;

root1 = 'Output_Files/';
%root1 = 'Figures/Runs_1_5/Helicopter_Noise/Taylor_Aero/Output_Files/';

file1 = [root1,'State.OUT'];

%root2 = 'Output_Files/Uncontrolled/';
%root2 = root1;
%root2 = 'Figures/Runs_1_5/Helicopter_Noise/Taylor_Aero/Output_Files/';
%file2 = [root2,'State222.2222.OUT'];

if COMPARE
    %LegendNames = {'Flaps','No Flaps'};
    LegendNames = {'Controlled','Uncontrolled'};
    %LegendNames = {'Baseline','Configuration 12'};
end

