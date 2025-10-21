%purge
close all

%fid = fopen('../Input_Files/Parafoils/Parafoil107_CradleNew_Panels.PAPA','r');
%fid = fopen('Input_Files/Parafoils/Parafoil1.2.PAPA','r');
fid = fopen('Input_Files/Parafoils/Airdrop_Canopy_Panels.PAPA','r');
%fid = fopen('Input_Files/Parafoils/SpirulineL.PAPA','r');

for ii = 1:9
    a = fgetl(fid);
end
str = fgetl(fid);
dx = str2num(str(1:find(str=='!')-1));
for ii = 1:16
    a = fgetl(fid);
end
rGC = [0;0;0];
str = fgetl(fid);
rGC(1) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGC(2) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGC(3) = str2num(str(1:find(str=='!')-1));

rGP = [0;0;0];
str = fgetl(fid);
rGP(1) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGP(2) = str2num(str(1:find(str=='!')-1));
str = fgetl(fid);
rGP(3) = str2num(str(1:find(str=='!')-1));

for ii = 1:27
    a = fgetl(fid);
end

npanels_str = fgetl(fid);
npanels = str2num(npanels_str(1:find(npanels_str=='!')-1));
NPANELS = npanels-2;

%%%Read in Area
ELES = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELES(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Stationline
ELESL = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELESL(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Buttline
ELEBL = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELEBL(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Waterline
ELEWL = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELEWL(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Phi
ELEPHI = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELEPHI(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in Theta
ELETHETA = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   ELETHETA(ii) = str2num(str(1:find(str=='!')-1));
end

%%Read in CL0
CL0 = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CL0(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in CLDELTA
CLDELTA = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CLDELTA(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in CLDELTA
CLDELTA3 = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CLDELTA3(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in CLA
CLA = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CLA(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in CD0
CD0 = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CD0(ii) = str2num(str(1:find(str=='!')-1));
end
%CD0_T = CD0(1)*(NPANELS/summCd0)^-1
%CD0(:) = CD0_T;

%%%Read in CDA2
CDA2 = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CDA2(ii) = str2num(str(1:find(str=='!')-1));
end
%CDA_T = CDA2(1)*(NPANELS*(alpha_P^2)/summCda)^-1
%CDA2(:) = CDA_T;

%%%Read in CDDELTA
CDDELTA = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CDDELTA(ii) = str2num(str(1:find(str=='!')-1));
end

%%%Read in CDDELTA3
CDDELTA3 = zeros(npanels,1);
for ii = 1:npanels
   str = fgetl(fid);
   CDDELTA3(ii) = str2num(str(1:find(str=='!')-1));
end

%%%%Create Lift and drag curves as a function of angle of attack
%and brake deflection

alfa = (0:1:18).*pi/180;
brake = 0:0.2:1.0;

panel = 4; %4=center panel,1=left panel

cl = zeros(length(alfa),length(brake));
cd = zeros(length(alfa),length(brake));

for ii = 1:length(alfa)
  for jj = 1:length(brake)
    cl(ii,jj) = CL0(panel) + CLA(panel)*alfa(ii) + CLDELTA(panel)*brake(jj) ...
	+ CLDELTA3(panel)*brake(jj)^3;
    cd(ii,jj) = CD0(panel) + CDA2(panel)*alfa(ii)^2 + ...
	CDDELTA(panel)*brake(jj) + CDDELTA3(panel)*brake(jj)^3;
    %%Stall model
    if (cl(ii,jj) < 0.0)
      cl(ii,jj) = 0.0;
    end
    if (alfa(ii) > 12*pi/180)
      cl(ii,jj) = cl(ii,jj) - 40.0*(alfa(ii)-12*pi/180)^2;
    end
  end
end

plottool(1,'Name',18,'Angle of Attack','CL');
plot(180/pi.*alfa,cl,'LineWidth',2)
legend(vec2cell(brake))
plottool(1,'Name',18,'Angle of Attack','CD');
plot(180/pi.*alfa,cd,'LineWidth',2)
legend(vec2cell(brake))

plottool(1,'CL and CD',18,'Angle of Attack(deg)','CL and CD');
p = [];
linestyle = {'k-','k-.','k-o','k--','k:','k-s'};
for ii = 1%:length(brake)
  p(ii) = plot(180/pi.*alfa,cl(:,ii),linestyle{ii},'LineWidth',2);
  plot(180/pi.*alfa,cd(:,ii),linestyle{ii+1},'LineWidth',2)
end
%legend(p,vec2cell(brake*100))

%%%This is actually the L/D curve of the main panel as a function of brake
%%%deflection
plottool(1,'L/D',18,'Angle of Attack(deg)','CL/CD');
ld = cl./cd;
ld_T = mean(ld')';
plot(180/pi.*alfa,ld,'LineWidth',2)
legend(vec2cell(brake*100))

fclose all;

