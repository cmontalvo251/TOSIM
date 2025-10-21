% purge
clear
close all
clc

PLOTFLIGHTDATA = 1;
PLOT6DATA = 0;
SIMULATE = 0;
PLAYMOVIE = 0;
PLAYSIMMOVIE = 0;

infile = '../Flight_Data/20140304_145426_TERN_Log.txt_Converted.txt';
file1 = 'Output_Files/State.OUT';
    
all = importdata(infile);

colheaders = all.textdata;

%%%Colheaders is pulled from the text file itself but I have listed them
%%%here for reference
%%%colheaders = {Seconds(s)	L_Length(m)	L_Tension(lbs)	 W_speed(m/s)
%%%W_dir(deg)	Lat(deg)	Long(deg)	Alt(m)	Hdg(deg)	Pitch(deg)
%%%Roll(deg)	Vx(m/s)	Vy(m/s)	Vz(m/s)	Att_err(deg)	Pos_err(m)	Vel_err(m/s)};

data = all.data;

time = data(:,1);
data = data(:,2:end);

if PLOTFLIGHTDATA
    for ii = 2:length(colheaders)
        plottool(1,colheaders{ii},18,colheaders{1},{colheaders{ii},'none'});
        plot(time,data(:,ii-1),'LineWidth',2)
    end
end

%%%Convert all this data to our coordinate system
tetherlength = data(:,1);
tensionlbs = data(:,2);
windspeed = data(:,3);
winddirection = data(:,4);
lat = data(:,5);
lon = data(:,6);
alt = data(:,7);
heading = data(:,8);
pitch = data(:,9);
roll = data(:,10);
Vx = data(:,11);
Vy = data(:,12);
Vz = data(:,13);
tstart = 0;
tend = time(end)-time(1);
tshift = time(1);

%%%%xyz
origin(1) = lat(1);
origin(2) = lon(1);
[x,y] = convertLATLON(lat,lon,origin);
z = -alt;
%%%ptp
phi = -roll*pi/180;
theta = -pitch*pi/180;
psi = -heading*pi/180;
psi = unwrap(psi);
%uvw
u = Vx;
v = Vy;
w = Vz;
%%%use phi,theta,psi to get p,q,r
[tp,p] = DerivativeFilter(phi,time,0.1,0);
[tq,q] = DerivativeFilter(theta,time,0.1,0);
[tr,r] = DerivativeFilter(psi,time,0.1,0);
%%%Convert tension
tension = tensionlbs*4.44;

%State Vector
state = [x,y,z,phi,theta,psi,u,v,w,p,q,r];

%%%Set end time of TERN.SIM
simname = 'Input_Files\TERN.SIM';
simdata = importdata(simname);
simdata{2} = [num2str(tend),'     !Final Time(sec)'];
writedata(simname,simdata);

%%%Assume we get prescribed motion of ship
filename = 'Input_Files\Ships\DDG_Flight_Data.SHIP';
fid = fopen(filename);
header = {};
for ii = 1:10
    header = [header;fgetl(fid)];
end

%%%%For now just set the heading of the ship equal to the heading of the cradle from the data
tship = linspace(tstart,tend,500)';
psiship = interp1(time-time(1),psi-psi(1),tship);
V0 = 12.86;
%%%Use x and y of cradle for now
%xship = interp1(time-time(1),x,tship);
%yship = interp1(time-time(1),y,tship);
%%%or integrate x and y
xship = 0*tship;
yship = 0*tship;
dt = tship(2)-tship(1);
for ii = 2:length(tship)
    xdot = V0*cos(psiship(ii));
    ydot = V0*sin(psiship(ii));
    xship(ii) = xship(ii-1) + xdot*dt;
    yship(ii) = yship(ii-1) + ydot*dt;
end
zship = 0*tship;
phiship = 0*tship;
thetaship = 0*tship;
uship = 0*tship+V0;
vship = 0*tship;
wship = 0*tship;
pship = 0*tship; 
qship = 0*tship;
rship = 0*tship;
shipdata = [tship,xship,yship,zship,phiship,thetaship,psiship,uship,vship,wship,pship,qship,rship];

tblsize = [num2str(length(tship)),'	!Number of Table Rows -> Columns are fixed - (T,x,y,z,phi,theta,psi,u,v,w,p,q,r)'];
writedata(filename,[header;tblsize]);
for ii = 1:length(shipdata)
    dlmwrite(filename,shipdata(ii,:),'delimiter',' ','precision',8,'-append')
end

if PLOT6DATA
    if SIMULATE
        system('Run.exe Input_Files/TERN.ifiles');
        [tZ] = dlmread(file1);
        t = tZ(:,1)+time(1);
        Z = tZ(:,2:end);
        [r,c] = size(Z);
        nT = round((c/2-13-18-12)/7)
        Zparafoil = Z(:,1:18);
        statesim = Zparafoil(:,[1,2,3,7,8,9,10,11,12,16,17,18]); %x,y,z,phic,thetac,psic,u,v,w,pc,qc,rc
        %%%shift x
        statesim(:,1) = statesim(:,1) - statesim(1,1) + state(1,1);
        %%%rotate heading
        statesim(:,6) = statesim(:,6) - statesim(1,6) + state(1,6);
        %%%%
        Plot6(1:12,{time,t},state,1,'m',0,{'Flight Data','Simulation Data'},{statesim});
        %%%Plot ship states
        Zship = Z(:,32:43);
        Plot6(1:12,{t},Zship,1,'m',0);
    else
        Plot6(1:12,{time},state,1,'m');
        plottool(1,'XY',18,'X (m)','Y (m)')
        plot(state(:,1),state(:,2),'k-','LineWidth',2)
    end
    tensionsim = Z(:,44 + 5 + (nT-1)*6 + 1);
    plottool(1,'Tension',18,'Time (sec)','Tension (N)')
    plot(time,tension,'k-','LineWidth',2)
    plot(t,tensionsim,'k--','LineWidth',2)
end

if PLAYMOVIE
    xmin = min(x);
    xmax = max(x);
    ymin = min(y);
    ymax = max(y);
    zmin = min(z);
    zmax = max(z);
    
    %%%Now make a video showing the cradle move around
    plottool(1,'Visualizer',18,'x(m)','y(m)','z(m)');
    for ii = 1:length(time)
        cla;
        CubeDraw(10,10,10,state(ii,1),state(ii,2),state(ii,3),state(ii,4),state(ii,5),state(ii,6),[1 0 1])
        %view(-50,28)
        view(0,90)
        title(num2str(time(ii)))
        xlim([xmin xmax])
        ylim([ymin ymax])
        zlim([zmin zmax])
        reverse('y')
        drawnow
    end
end

if PLAYSIMMOVIE

    if ~exist('statesim','var')
        [tZ] = dlmread(file1);
        t = tZ(:,1)+time(1);
        Z = tZ(:,2:end);
        [r,c] = size(Z);
        nT = round((c/2-13-18-12)/7)
        Zparafoil = Z(:,1:18);
        statesim = Zparafoil(:,[1,2,3,7,8,9,10,11,12,16,17,18]); %x,y,z,phic,thetac,psic,u,v,w,pc,qc,rc
        %%%shift x
        statesim(:,1) = statesim(:,1) - statesim(1,1) + state(1,1);
        %%%rotate heading
        statesim(:,6) = statesim(:,6) - statesim(1,6) + state(1,6);
    end
    
    xsim = statesim(:,1);
    ysim = statesim(:,2);
    zsim = statesim(:,3);
    xmin = min(xsim);
    xmax = max(xsim);
    ymin = min(ysim);
    ymax = max(ysim);
    zmin = min(zsim);
    zmax = max(zsim);
    
    %%%Now make a video showing the cradle move around
    plottool(1,'Visualizer',18,'x(m)','y(m)','z(m)');
    for ii = 1:length(time)
        cla;
        CubeDraw(10,10,10,statesim(ii,1),statesim(ii,2),statesim(ii,3),statesim(ii,4),statesim(ii,5),statesim(ii,6),[0 1 1])
        %view(-50,28)
        view(0,90)
        title(num2str(time(ii)))
        xlim([xmin xmax])
        ylim([ymin ymax])
        zlim([zmin zmax])
        reverse('y')
        drawnow
    end
end

    
