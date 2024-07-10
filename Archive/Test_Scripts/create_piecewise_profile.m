function create_piecewise_profile(L0,Lf,Ldotmax,tf)

close all


% Extension
t0 = 0;
t_vec1 = linspace(0,tf/2,100);
t_vec2 = linspace(tf/2,tf,100);
Lreel1 = 0*t_vec1;
Ldot1 = Lreel1;

%%%% a0 + a1*t + a2*t^2 + a3*t^3 - third order
%%%% L(t = 0) = a0 = L0 thus a0 = L0
%%%% Lprime(t = 0) = 0 = a1 + 2*a2*t + 3*a3*t^2 = a1 thus a1 = 0;
%%%% Lprime(t = tf/2) = a2*tf + (3/4)*a3*tf^2 = Ldotmax
%%%% L(t = tf/2) = L0 + a2*(tf/2)^2 + a3*(tf/2)^3 = (Lf+L0)/2

%%%% 2 eqns [tf tf^2*(3/4)]*x = Ldotmax
          % [(tf/2)^2 (tf/2)^3]*x = (Lf+L0)/2-L0

A = [tf tf^2*(3/4);(tf/2)^2 (tf/2)^3];
B = [Ldotmax;(Lf+L0)/2-L0];
x = inv(A)*B;
          
a0 = L0;
a1 = 0;
a2 = x(1);
a3 = x(2);
Lreel1 = a0 + a1*t_vec1 + a2*t_vec1.^2 + a3*t_vec1.^3;
Ldot1 = a1 + 2*a2*t_vec1 + 3*a3*t_vec1.^2;

%%%%Second Half
%%% b0 + b1*(t-tf/2) + b2*(t-tf/2)^2 + b3*(t-tf/2)^3
%%% L(t = tf/2) = (Lf+L0)/2 = b0 thus b0 = (Lf+L0)/2
%%% Lprime(t = tf/2) = Ldotmax = b1 
%%% Lprime(t = tf) = 0 = Ldotmax + 2*b2*(tf-tf/2) + 3*b3*(tf-tf/2)^2
%%% L(t = tf) = Lf = b0 + b1*(tf-tf/2) + b2*(tf-tf/2)^2 + b3*(tf-tf/2)^2
b0 = (Lf+L0)/2;
b1 = Ldotmax;

A = [2*(tf-tf/2) 3*(tf-tf/2)^2;(tf-tf/2)^2 (tf-tf/2)^2];
B = [-Ldotmax; -b0-b1*(tf-tf/2)];

x = inv(A)*B;
b2 = x(1);
b3 = x(2);

%Lreel2 = b0 + b1*(t_vec2-tf/2) + b2*(t_vec2-tf/2).^2 + b3*(t_vec2-tf/2).^3;
%Ldot2 = b1 + 2*b2*(t_vec2-tf/2) + 3*b3*(t_vec2-tf/2).^2;

Lreel2 = -Lreel1(end:-1:1)+2*b0;
Ldot2 = Ldot1(end:-1:1);

Lreel = [Lreel1 Lreel2];
Ldot = [Ldot1 Ldot2];
t_vec = [t_vec1 t_vec2];

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

