%function Visualizer(t,Z,nT,pship,pparafoil,paircraft,ptether)
%close all
purge

parameters

%%%%IMPORT DATA
if ~exist('tZ','var')
   
    %[tZ] = dlmread('Output_Files/State_Extend_Retract_WRF_Ship_Airwake.OUT');
    [tZ] = dlmread('Output_Files/State.OUT');
    
    %tZ = dlmread('../Example_Output_Files/State_extension_retraction.OUT');
    %tZ = dlmread('../Example_Output_Files/Trimming_Parafoil107.OUT');
    %tZ = dlmread('../Example_Output_Files/Flying_Trimmed_In_13ms_SideWind.OUT');
    %tZ = dlmread('../Example_Output_Files/SideWinds/SideWinds_3ms_On_Off.OUT');
    %tZ = dlmread('../Example_Output_Files/Ext_Ret_Winds/State_reelout_reelin_WRF.OUT');
end

%dlmwrite('Truncated.OUT',tZ(1:100:end,:),'delimiter',' ')

t = tZ(:,1);
Z = tZ(:,2:end);
[r,c] = size(Z);
nT = round((c/2-13-18-12)/7)
     
%%%This code assumes the following
%Z = [parafoil_states(18),aircraft(13),ship(12),tether(6*nT)]

plottool(1,'Visualization',12,'x','y','z');
%%%Isometric
%Az = 50; 
%El = 16;
%%%%XZ plane
Az = 0;
El = 0;
%%%%XY Plane
%Az = 0;
%El = 90;

tstart = 900;
istart = find(t > tstart,1);

for ii = istart:skip:length(t)
    hold off
    cla;
    Zrow = Z(ii,:);
    if vparafoil
        %%%Parafoil
        dx = 2.1;dy = 8.8;dz = dx*0.15;
        %%%Position of parafoil
        ptp = Zrow(4:6);
        Incidence = 0.0610;
        Rparafoil = R123(ptp(1),ptp(2),ptp(3));
        rpshift = 0*Zrow(1:3) + (Rparafoil*[-dx/4;0;-5.75])';
        rpfront = 0*Zrow(1:3) + (Rparafoil*[dx/4;0;-5.75])';
        rprear = 0*Zrow(1:3) + (Rparafoil*[-3*dx/4;0;-5.75])';
        rp = 0*Zrow(1:3) + (Rparafoil*[0;0;-5.75])';
        CubeDraw(dx,dy,dz,rpshift(1),rpshift(2),rpshift(3),Zrow(4),Zrow(5)+Incidence,Zrow(6),[1 0 0]);
        hold on
        %%%Cradle
        dx = 0.6097;dy = 0.6098;dz = 0.7113;
        %%%Position of cradle
        ptp = Zrow(7:9);
        Rcradle = R123(ptp(1),ptp(2),ptp(3));
        rc = 0*Zrow(1:3) + (Rcradle*[0;0;0.470])';
        CubeDraw(dx,dy,dz,rc(1),rc(2),rc(3),Zrow(7),Zrow(8),Zrow(9),[0 1 0]);
        %%%%Position of Gimbal
        [x,y,z] = sphere();
        surf(0.1.*x,0.1*y,0.1*z);
        %%%%Plot Aero center
        [x,y,z] = sphere();
        surf(0.25.*x,0.25*y,0.25*z-5.75);
        %%%Plot line from cg to rp
        plot3([0 rc(1)],[0 rc(2)],[0 rc(3)],'k-','LineWidth',1)
        plot3([0 rpfront(1)],[0 rp(2)],[0 rp(3)],'k-','LineWidth',1)
        plot3([0 rprear(1)],[0 rprear(2)],[0 rprear(3)],'k-','LineWidth',1)
    end
    xlim([-5 5])
    ylim([-5 5])
    zlim([-8 2])
    %%%%
    title(num2str(t(ii)))
    hold on
    reverse(['y','z'])
    xlabel('x(m)')
    ylabel('y')
    zlabel('z(m)')
    axis square
    axis on
    grid on
    view(Az,El)
    light('Position',-1000.*[0 0 2],'Style','infinite');
    drawnow
end


