%%%%Create Ship Motion Table Data
purge

filename = 'Accel.HELI';
outfilename = 'Turn.HELI';

fid = fopen(filename);

header = {};
for ii = 1:11
    header = [header;fgetl(fid)];
end

ACCEL = 0;
SHAKEY = 0;
TURN = 1;

tabsize = 500;

tfinal = 200;

tout = (linspace(0,tfinal,tabsize))';

if TURN
    turnrate = 5; %%%deg/sec
    turn_time = 360/turnrate; %%sec
    tout = (linspace(0,turn_time,tabsize))';
    umax = 101.60721;
    distance = umax*turn_time;
    radius = distance/(2*pi);
    
    psi = linspace(0,2*pi,tabsize)';

    x = radius*sin(psi);
    y = -radius*cos(psi)+radius;
    z = zeros(length(tout),1);
    phi = zeros(length(tout),1);
    theta = phi;
    u = zeros(length(tout),1)+umax;
    v = zeros(length(tout),1);
    w = zeros(length(tout),1);
    p = phi;
    q = theta;
    r = 0*psi+turnrate*pi/180;
end

if ACCEL
    x = zeros(length(tout),1);
    y = x;
    z = y;
    phi = z;
    theta = phi;
    psi = theta;
    u = psi;
    v = psi;
    w = psi;
    p = psi;
    q = psi;
    r = psi;
    
    umax = 101.404;
    dt = tout(2)-tout(1);
    a = psi;
    F = psi;
    kp = 1.0;
    s = 0.1;
    s2 = 1.0;
    %%%Integrate
%     tout(1) = 0;
%     a(1) = kp;
%     for idx = 1:length(tout)-1
%         x(idx+1) = x(idx) + u(idx)*dt;
%         if u(idx) > umax
%             f = 0;
%         else
%             f = sign(umax-u(idx));
%         end
%         F(idx+1) = (1-s2)*F(idx) + s2*kp*f;
%         a(idx+1) = (1-s)*a(idx) + s*F(idx+1);
%         u(idx+1) = u(idx) + a(idx+1)*dt;
%         tout(idx+1) = tout(idx) + dt;
%     end
    %%%Try this next - use x = (1/2)*a*t^2 and v = a*t but smooth out the 
    %%%velocity curve
    
    tmax = 100;
    
    loc = find(tout > tmax,1);
    
    tloc = tout(loc);
    
    %a = umax/tloc;
    
    %u(1:loc) = a*tout(1:loc);
    %x(1:loc) = (1/2)*a*tout(1:loc).^2;

    %u(loc+1:end) = umax;
    %x(loc+1:end) = x(loc) + umax*(tout(loc+1:end)-tmax);
    
    a = umax/tmax;
    x(1:loc) = (1/2)*a*tout(1:loc).^2;
    u(1:loc) = a*tout(1:loc);
    x(loc+1:end) = x(loc) + u(loc)*(tout(loc+1:end)-tout(loc));
    u(loc+1:end) = u(loc);
    
    plottool(1,'x',18,'Time (sec)','X (ft)');
    plot(tout,x)
    plottool(1,'u',18,'Time (sec)','U (ft/s)');
    plot(tout,u,'LineWidth',2)
    plottool(1,'x',18,'Time (sec)','Acceleration (ft/s^2)');
    plot(tout,a,'LineWidth',2)
end

if SHAKEY
    x = 0*rand(length(tout),1)+101.04*tout;
    y = 10*(rand(length(tout),1)-0.5);
    z = 10*(rand(length(tout),1)-0.5);
    phi = zeros(length(tout),1);
    theta = phi;
    psi = theta;
    u = 1*rand(length(tout),1)+101.04;
    v = 1*rand(length(tout),1);
    w = 1*rand(length(tout),1);
    p = phi;
    q = theta;
    r = psi;
end

data = [tout,x,y,z,phi,theta,psi,u,v,w,p,q,r];

Plot6([1:12],{tout},data(:,2:end),1,'ft',0);
plottool(1,'XvsY',12,'X','Y')
plot(data(:,2),data(:,3),'k-','LineWidth',2)
reverse('y')

[r,c] = size(data)

header{end} = [num2str(tabsize),' !Table Size'];

writedata(outfilename,header);
for ii = 1:length(data)
    dlmwrite(outfilename,data(ii,:),'delimiter',' ','precision',8,'-append')
end

fclose all;

tend = data(end,1)