function [SLCG,WLCG,BLCG] = getTetherConnectionPointHeli()
helifile = gethelifile();
%str = ['fid = fopen(''',helifile,''');']
%eval(str)
fid = fopen(helifile);

for idx = 1:10
    d = fgetl(fid);
end

% Stationline, buttline and waterline begins at 10th line in quad file
SLCG = fgetl(fid);
SLCG = str2num(SLCG(1:find(SLCG=='!')-1));
BLCG = fgetl(fid);
BLCG = str2num(BLCG(1:find(BLCG=='!')-1));
WLCG = fgetl(fid);
WLCG = str2num(WLCG(1:find(WLCG=='!')-1));

% Hardcoded for the Quadcopter
% heliparams = importdata('Input_Files/Copters/Quad.COPTER');
% SLCG = heliparams{11};
% SLCG = str2num(SLCG(1:find(SLCG=='!')-1));
% BLCG = heliparams{12};
% BLCG = str2num(BLCG(1:find(BLCG=='!')-1));
% WLCG = heliparams{13};
% WLCG = str2num(WLCG(1:find(WLCG=='!')-1));