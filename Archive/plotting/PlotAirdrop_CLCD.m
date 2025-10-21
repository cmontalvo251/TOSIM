purge

sig1 = [0.25 1.5];
cd0 = [0.1373 0.0351];
cda2 = [0.0977 3.9642];
cl0 = [-0.2636 0.1313];
cla = [2.8139 1.9825];

dS = linspace(0.25,1,10);
aoa = linspace(0,15*pi/180,10);

[hcl,fcl] = plottool(1,'CL',18,'AOA (deg)','CL');
[hcd,fcd] = plottool(1,'CD',18,'AOA (deg)','CD');

cl_mat = zeros(length(dS),length(aoa));
cd_mat = zeros(length(dS),length(aoa));

for ii = 1:length(dS)
    cd0i = interp1(sig1,cd0,dS(ii),'linear','extrap');
    cda2i = interp1(sig1,cda2,dS(ii),'linear','extrap');
    cl0i = interp1(sig1,cl0,dS(ii),'linear','extrap');
    clai = interp1(sig1,cla,dS(ii),'linear','extrap');
    
    cl = cl0i + clai.*aoa;
    cd = cd0i + cda2i.*(aoa.^2);
    plot(fcl,aoa.*180/pi,cl)
    plot(fcd,aoa.*180/pi,cd)
    
    cl_mat(ii,:) = cl;
    cd_mat(ii,:) = cd;
end

%%%1-D Fitting Procedure
cli = cl_mat(:,1);
aoai = aoa(1);

X = cli - cl0(1) - cla(1)*aoai;
dS = dS';
H = [(dS-0.25)];
theta = inv(H'*H)*H'*X;
cld = theta(1);
clfit = cl0(1) + cla(1)*aoai + cld*(dS-0.25);

plottool(1,'CL',18,'dS','CL')
plot(dS,cli)
plot(dS,clfit,'r-')

%%%2-D Fitting Procedure
X_mat = [];
H_mat = [];
for ii = 1:length(aoa)
    cli = cl_mat(:,ii);
    aoai = aoa(ii);
    Xi = cli - cl0(1) - cla(1)*aoai;
    Hi = (dS-0.25);
    X_mat = [X_mat;Xi];
    H_mat = [H_mat;Hi];
end
theta = inv(H_mat'*H_mat)*H_mat'*X_mat;
cld_mat = theta(1);
cl_mat_fit = 0.*cl_mat;

for ii = 1:length(dS)
    cl = cl0(1) + cla(1).*aoa + cld_mat*(dS(ii)-0.25);
    cl_mat_fit(ii,:) = cl;
end    

plottool(1,'CL',18,'AOA (deg)','dS','CL');
[xx,yy] = meshgrid(aoa.*180/pi,dS);
mesh(xx,yy,cl_mat)
surf(xx,yy,cl_mat_fit)

%%%Now Same for CD
%%%1-D Fitting Procedure
cdi = cd_mat(:,1);
aoai = aoa(1);

X = cdi - cd0(1) - cda2(1)*aoai^2;
H = [(dS-0.25)];
theta = inv(H'*H)*H'*X;
cdd = theta(1);
cdfit = cd0(1) + cda2(1)*aoai^2 + cdd*(dS-0.25);

plottool(1,'CD',18,'dS','CD');
plot(dS,cdi)
plot(dS,cdfit,'r-')

%%%2-D Fitting Procedure
X_mat = [];
H_mat = [];
for ii = 1:length(aoa)
    cdi = cd_mat(:,ii);
    aoai = aoa(ii);
    Xi = cdi - cd0(1) - 0.56*aoai^2;
    Hi = [(dS-0.25) (dS-0.25).^3];
    X_mat = [X_mat;Xi];
    H_mat = [H_mat;Hi];
end
theta = inv(H_mat'*H_mat)*H_mat'*X_mat;
cdd_mat = theta(1);
cdd3_mat = theta(2);
cd_mat_fit = 0.*cd_mat;

for ii = 1:length(dS)
    cd = cd0(1) + 0.56.*aoa.^2 + cdd_mat*(dS(ii)-0.25) + cdd3_mat*(dS(ii)-0.25)^3;
    cd_mat_fit(ii,:) = cd;
end    

plottool(1,'CD',18,'AOA (deg)','dS','CD');
[xx,yy] = meshgrid(aoa.*180/pi,dS);
mesh(xx,yy,cd_mat)
surf(xx,yy,cd_mat_fit)


