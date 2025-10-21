function [SLCG,WLCG,BLCG] = getTetherConnectionPoint()
tether_params = importdata('Input_Files/Tethers/TOSIM.THR');
SLCG = tether_params{23};
SLCG = str2num(SLCG(1:find(SLCG=='!')-1));
BLCG = tether_params{24};
BLCG = str2num(BLCG(1:find(BLCG=='!')-1));
WLCG = tether_params{25};
WLCG = str2num(WLCG(1:find(WLCG=='!')-1));