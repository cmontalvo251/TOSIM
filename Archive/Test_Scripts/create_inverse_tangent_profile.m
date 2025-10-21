function create_inverse_tangent_profile(L0,Lf,Ldotmax,tf)

close all


% Extension
t0 = 0;
t_vec = linspace(0,tf,200);
%sig = 0.05;
dL = (Lf-L0);
tm = (tf + t0)/2;
sig = fzero(@(sig) myfun(sig,dL,tm,Ldotmax),0.05)
%sig = 0.05;
%myfun(sig,dL,tm,Ldotmax)
arg = sig*(t_vec-tm); %%%arg goes from -tm to tm arg0 = -tm*S
offset = abs(atan(-tm*sig));
stretch = dL/(2*offset);
Lreel = stretch*(atan(arg)+offset)+L0;
%Lreel = (Lf-L0)*atan(sig*(t_vec-tm))/(2*(atan(sig*tm))) + (Lf+L0)/2;
x = sig*(t_vec-tm);
Ldot = dL*sig/(2*atan(sig*tm))*(1./(1+x.^2));
%Ldotmax = dL*sig/(2*atan(sig*tm));

%figure
plottool(1,'Tether',18);
plot(t_vec,Lreel,'k-','LineWidth',2)
xlabel('Time (s)')
ylabel('Tether Length (ft)')

plottool(1,'TetherDot',18)
plot(t_vec,Ldot,'k-','LineWidth',2)
xlabel('Time (s)')
ylabel('Tether Reel Out Rate (ft/s)')

return

outfilename = 'Input_Files/Tethers/PayOut.TCOM'

FID = fopen(outfilename,'w');
for i = 1:numinterp
    fprintf(FID,'%.10f\n',tvecout(i)) ;
end
for i = 1:numinterp
    fprintf(FID,'%.10f\n',Lint(i)) ;
end
fclose(FID);

function f = myfun(sig,dL,tm,Ldotmax)
f = dL/(2*Ldotmax)*sig - atan(tm*sig);

