purge

%%%SWH = 1.25,4.0,6.0 m = 4.1, 13.12, 19.68 ft
%%%Beaufort Scale - Sea State = 3,5, and 6
%%%Beaufort Scale - Wind speed = 15, 25, 30 kts = 7.72,12.86,15.4 m/s
%%%Wind Scale = 1.9,3.2,3.9

%%%AR3.ASC = Ship Speed = 20 knots, Sea State = 3
%%%AR7.ASC = Ship Speed = 20 knots, SWH = 4.0 > SS = 5
%%%AR11.ASC = Ship Speed = 20 knots, SWH = 6.0 > SS = 6

infile = 'AR11.ASC'
Vnominal = (1/3.28)*20*1.15*5280/3600
Heading = 0;

all = importdata(infile,' ',6);
data = all.data(:,2:end);

t = data(:,1);
l = ones(length(t),1);
uvw = data(:,15:17)+l*[Vnominal,0,0];
dxyz = data(:,3:5);
xyz = dxyz;
ptp = data(:,6:8).*pi/180;
%%%assume x = cos(heading)*Vnominal*t + dx
xyz(:,1) = cos(Heading).*Vnominal.*t + dxyz(:,1);
xyz(:,2) = sin(Heading).*Vnominal.*t + dxyz(:,2);
xyz(:,3) = dxyz(:,3);
pqr = data(:,9:11).*pi/180;
state = [xyz,ptp,uvw,pqr];
Plot6([1:12],{t},state,1,'m',0);

%%%Now create the input deck
filename = 'TAB.SHIP';

fid = fopen(filename);

header = {};
for ii = 1:10
    header = [header;fgetl(fid)];
end
fclose(fid);
TABSIZE = 500;
TEND = 500;

L = find(t > TEND,1);
if isempty(L)
  L = length(t);
end

skip = round(L/TABSIZE);

fid = fopen(outfile,'w');
for ii = 1:length(header)
    fprintf(fid,'%s \n',header{ii});
end
fprintf(fid,'%s \n',[num2str(TABSIZE),' !Table Size']);
for ii = 1:skip:L
  fprintf(fid,'%s \n',num2str([t(ii),state(ii,:)]));
end
fclose(fid);

disp('File created')



