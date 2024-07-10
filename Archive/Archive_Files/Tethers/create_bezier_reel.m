function create_bezier_reel(L0,Lf,Ldotmax,tf)
close all

x = linspace(0,1,200);

%%%P0 and P3 are fixed points
P0 = [0 L0]';
P3 = [tf Lf]';

%%%P1 and P2 are determined by maximum slope
%dt = (Lf-L0)/Ldotmax;
t1 = tf - (Lf-L0)/Ldotmax;
t2 = 2*(Lf-L0)/Ldotmax + t1 - tf;

%%%t1 and t2 are placed in the middle of the curve
%t1 = (tf-dt)/2;
%t2 = t1 + dt;

P1 = [t1;L0];
P2 = [t2;Lf];

B = zeros(2,length(x));
dBdx1 = B;
dBdx2 = B;
for idx = 1:length(x)
    xi = x(idx);
    B(:,idx) = (1-xi)^3*P0 + 3*(1-xi)^2*xi*P1 + 3*(1-xi)*xi^2*P2 + xi^3*P3;
    dBdx1(:,idx) = 3*(1-xi)^2*(P1-P0) + 6*(1-xi)*xi*(P2-P1) + 3*xi^2*(P3-P2);
    dBdx2(:,idx) = -3*(P0*(-1+xi)^2+xi*(-2*P2+3*P2*xi-P3*xi)+P1*(-1+4*xi-3*xi^2));
end
figure()
plottool(1,'Bezier Derivative',18,'x(0-1)','dBdx');
plot(dBdx1')
legend('dtdx','dLdx')

t_vec = B(1,:);
Lreel = B(2,:);
dtdx = dBdx1(1,:);
dLdx = dBdx1(2,:);
dLdt = dLdx./dtdx;

plottool(1,'TetherDot',18);
plot(t_vec,dLdt,'k-','LineWidth',2)
xlabel('Time (s)')
ylabel('Tether Reel Out Rate (ft/s)')

%figure
plottool(1,'Tether',18);
plot(P0(1),P0(2),'r*','MarkerSize',20)
plot(P1(1),P1(2),'r*','MarkerSize',20)
plot(P2(1),P2(2),'r*','MarkerSize',20)
plot(P3(1),P3(2),'r*','MarkerSize',20)
plot(t_vec,Lreel,'k-','LineWidth',2)
xlabel('Time (s)')
ylabel('Tether Length (ft)')

if min(t_vec) < 0
    tfmin = (Lf-L0)/Ldotmax;
    disp(['Minimum tf is = ',num2str(tfmin)])
    error('Time vector is negative. Run program again with larger tf or larger Ldotmax')
    return
end

outfilename = 'Input_Files/Tethers/PayOut.TCOM'

FID = fopen(outfilename,'w');
numinterp = length(t_vec)
for i = 1:numinterp
    fprintf(FID,'%.10f\n',t_vec(i)) ;
end
for i = 1:numinterp
    fprintf(FID,'%.10f\n',Lreel(i)) ;
end
fclose(FID);




