%%%%Plot Forces
close all
clc
parameters;

data = dlmread('Output_Files/Forces.OUT');

[r,c] = size(data);

nbeads = round((c-1)/(9));

time = data(:,1);

if ptether
    L = 3*nbeads;
    grav = data(:,2:(L+1));
    elas = data(:,(L+2):(2*L+1));
    aero = data(:,(2*L+2):(3*L+1));

    [hgrav,axgrav] = plottool(1,'Gravity',18,'Time (sec)','Force Gravity (lbf)');
    [helas,axelas] = plottool(1,'Elasticity',18,'Time (sec)','Force Elasticity (lbf)');
    [haero,axaero] = plottool(1,'Aero',18,'Time (sec)','Force Aerodynamics (lbf)');

    color = {'-','--','-.',':','-','--','-.',':','-','--','-.',':'};

    ctr = 0;
    for idx = 1:nbeads
        ctr = ctr + 1;
        if ctr > length(color)
           ctr = 1;
        end
        sumFx = 0;
        sumFy = 0;
        sumFz = 0;

        s = idx;
        fx = grav(:,s);
        fy = grav(:,s+nbeads);
        fz = grav(:,s+2*nbeads);
        sumFx = sumFx + fx(end);
        sumFy = sumFy + fy(end);
        sumFz = sumFz + fz(end);

        plot(axgrav,time,fx,[color{ctr},'b'],'LineWidth',2);
        plot(axgrav,time,fy,[color{ctr},'r'],'LineWidth',2);
        plot(axgrav,time,fz,[color{ctr},'g'],'LineWidth',2);

        fx = elas(:,s);
        fy = elas(:,s+nbeads);
        fz = elas(:,s+2*nbeads);
        sumFx = sumFx + fx(end);
        sumFy = sumFy + fy(end);
        sumFz = sumFz + fz(end);
        plot(axelas,time,fx,[color{ctr},'b'],'LineWidth',2);
        plot(axelas,time,fy,[color{ctr},'r'],'LineWidth',2);
        plot(axelas,time,fz,[color{ctr},'g'],'LineWidth',2);

        fx = aero(:,s);
        fy = aero(:,s+nbeads);
        fz = aero(:,s+2*nbeads);
        sumFx = sumFx + fx(end);
        sumFy = sumFy + fy(end);
        sumFz = sumFz + fz(end);
        plot(axaero,time,fx,[color{ctr},'b'],'LineWidth',2);
        plot(axaero,time,fy,[color{ctr},'r'],'LineWidth',2);
        plot(axaero,time,fz,[color{ctr},'g'],'LineWidth',2);

        disp(num2str([sumFx,sumFy,sumFz]))

    end


    legend(axgrav,'Fx','Fy','Fz')
    legend(axelas,'Fx','Fy','Fz')
    legend(axaero,'Fx','Fy','Fz')

end

%%%%TOWED BODY FORCES
if ptowed
    data = dlmread('Output_Files/Misc.OUT');
    time = data(:,1)-50;
    FXYZ = data(:,6:8);
    MXYZ = data(:,9:11);
    plottool(1,'Forces Towed Body',18,'Time (sec)','Connection Point Forces (lbf)');
    plot(time,FXYZ,'LineWidth',2)
    legend('Fx','Fy','Fz')
    disp('Std of Forces')
    std(FXYZ)
    xlim([0 50])
    plottool(1,'Moments Towed Body',18,'Time (sec)','Connection Point Moments (lbf)');
    plot(time,MXYZ,'LineWidth',2)
    legend('Fx','Fy','Fz')
    disp('Std of Moments')
    std(MXYZ)
    xlim([0 50])
end

if pheli
    k = c - 23;   % Starting index for quad forces
    data = dlmread('Output_Files/Forces.OUT');
    time = data(:,1);
    QUAD_FXYZtotal = data(:,k:k+2);
    QUAD_FXYZgrav = data(:,k+3:k+5);
    QUAD_FXYZaero = data(:,k+6:k+8);
    QUAD_FXYZcont = data(:,k+9:k+11);
    
    TOW_FXYZtotal = data(:,k+12:k+14);
    TOW_FXYZgrav = data(:,k+15:k+17);
    TOW_FXYZaero = data(:,k+18:k+20);
    TOW_FXYZcont = data(:,k+21:k+23);
    
    plottool(1,'Quad Total Forces',18,'Time (s)','Forces (lbf)');
    plot(time,QUAD_FXYZtotal)
    legend('Fx','Fy','Fz')
    
    plottool(1,'Quad Grav Forces',18,'Time (s)','Forces (lbf)');
    plot(time,QUAD_FXYZgrav)
    legend('Fx','Fy','Fz')
    
    plottool(1,'Quad Aero Forces',18,'Time (s)','Forces (lbf)');
    plot(time,QUAD_FXYZaero)
    legend('Fx','Fy','Fz')
    
    plottool(1,'Quad Contact Forces',18,'Time (s)','Forces (lbf)');
    plot(time,QUAD_FXYZcont)
    legend('Fx','Fy','Fz')
    
    %----------------------------------------------------------------
    
    plottool(1,'Tow Total Forces',18,'Time (s)','Forces (lbf)');
    plot(time,TOW_FXYZtotal)
    legend('Fx','Fy','Fz')
    
    plottool(1,'Tow Grav Forces',18,'Time (s)','Forces (lbf)');
    plot(time,TOW_FXYZgrav)
    legend('Fx','Fy','Fz')
    
    plottool(1,'Tow Aero Forces',18,'Time (s)','Forces (lbf)');
    plot(time,TOW_FXYZaero)
    legend('Fx','Fy','Fz')
    
    plottool(1,'Tow Contact Forces',18,'Time (s)','Forces (lbf)');
    plot(time,TOW_FXYZcont)
    legend('Fx','Fy','Fz')
end
