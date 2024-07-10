function create_bezier_profile_in_out_in(dout,din, tout,tin, tmid)
close all
tstart = 0 ;

if tmid == 0
    tmid = 1e-10;
end

%tdur = 300 ;
tend = tstart+tout; 

Lstart = 196; %%30
Lstart_copy = Lstart ;

Lend = 30; %%50

tauvec = linspace(0,1,150) ;

% Extension
d = dout;
for i = 1:numel(tauvec)
    
    tau = tauvec(i) ;
    tb(i) = (1-tau)^3*tstart + 3*tau*(1-tau)^2*(tstart+d) + 3*tau^2*(1-tau)*(tend-d) + tau^3*tend ;
    Lreel(i) = (1-tau)^3*Lstart + 3*tau*(1-tau)^2*Lstart + 3*tau^2*(1-tau)*Lend + tau^3*Lend ;
       
end

% Retraction
d = din;
Lstart = Lend ;
Lend = Lstart_copy ;
tstart = tend + tmid ;
tend = tstart + tin ;
for i = 1:numel(tauvec)
    
    tau = tauvec(i) ;
    tb_in(i) = (1-tau)^3*tstart + 3*tau*(1-tau)^2*(tstart+d) + 3*tau^2*(1-tau)*(tend-d) + tau^3*tend ;
    Lreel_in(i) = (1-tau)^3*Lstart + 3*tau*(1-tau)^2*Lstart + 3*tau^2*(1-tau)*Lend + tau^3*Lend ;
       
end

tb = [tb tb_in] ;
Lreel = [Lreel Lreel_in] ;

% Output table of data for use in controller
numinterp = 200 ;
tvecout = linspace(tb(1),tb(numel(tb)),numinterp) ;
Lint = interp1(tb,Lreel,tvecout);
plottool(1,'Tether',18);
plot(tvecout,Lint,'k-','LineWidth',2)
xlabel('Time (s)')
ylabel('Tether Length (ft)')

outfilename = 'Input_Files/Tethers/PayOutIn.TCOM'

FID = fopen(outfilename,'w');
for i = 1:numinterp
    fprintf(FID,'%.10f\n',tvecout(i)) ;
end
for i = 1:numinterp
    fprintf(FID,'%.10f\n',Lint(i)) ;
end
fclose(FID);

%figure

% plot([tstart tstart+d],[Lstart Lstart],'k','LineWidth',2);
% plot([tend tend-d],[Lend Lend],'k','LineWidth',2);
% plot([tstart+d tend-d],[Lstart Lend],'or','LineWidth',2,'MarkerSize',8)
% plot([0 tstart],[Lstart Lstart],'LineWidth',2);
% plot([tend tend+100],[Lend Lend],'LineWidth',2);
% axis([0 tend+50 0 350])






