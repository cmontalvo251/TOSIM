clear
close all
% clc

parameters

if pforces
    plotForces
end
if psweeps
    plotSweeps
end

%%%%IMPORT DATA
if ~exist('tZ','var')
    [tZ] = dlmread(file1);
end
if COMPARE
    tZother = dlmread(file2);
    tOther = tZother(:,1);
    Zother = tZother(:,2:end);
end

t = tZ(:,1)+COMPARE*OFFSET;
Z = tZ(:,2:end);
[r,c] = size(Z);
nT = round((c/2-13-12)/7); 
nostates = 13+12+7*nT+1;

%%%%Limit time
tstart = 0;
l = find(t >= tstart,1);
ff = find(t >= 5);
t = t(l:end);
Z = Z(l:end,:);

if ptether || ptowed
    %%%%Get SLCG
    [SLCG,WLCG,BLCG] = getTetherConnectionPoint();
    SLCG_HELI = 0;
    WLCG_HELI = 0;
    BLCG_HELI = 0;
%     [SLCG_HELI,WLCG_HELI,BLCG_HELI] = getTetherConnectionPointHeli();
end

%%%%%% Note that this plots all angles in ft as well... I believe. This perhaps can mess up
%%%%%% the units whenever we're looking at it.
%%%%Plot states
if pheli
    Zheli = Z(:,14:25);
    Zheli(:,3) = -1*Z(:,16); %%% REVISIT Multiplied by -1 so z-coordinate shows altitude. - N.H. 5/2/17
    Zdot = Z(:,nostates+1:end);
    Zhelidot = Zdot(:,14:25);
    
    %%% REVISIT
    if COMPARE
        Zsother = Zother(:,14:25);
        Plot6(1:12,{t,tOther},Zheli,1,'ft',0,{'1','2'},{Zsother});
    else
        Plot6([1:12],{t},Zheli,1,'ft',0,'',[],[],'Quadrotor');
        
        
%         figure()
%         plot(t,Zheli(:,3))
%         xlabel('Time (s)')
%         ylabel('z(ft) Quadrotor')
%         ylim([12,22])
%         figure()
%         plot(t,Zheli(:,3),'k-','LineWidth',2)
%         hold on
%         plot([16 16],[10 87.5],'r','LineWidth',1.75)
%         plot([t(1),t(end)],[85,85],'r--','LineWidth',1.75)
%         title('Z')
%         xlabel('Time (s)')
%         ylabel('Altitude (ft.)')
        
%         figure()
%         plot(t,Zheli(:,9),'k-','LineWidth',2)
%         title('w')
%         xlabel('Time (s)')
%         ylabel('Altitude/s (ft./s)')
        
%%% REVISIT UNCOMMENT
%         figure()
%         hold on 
%         plot(Zheli(1,2),Zheli(1,1),'ko','MarkerSize',8,'MarkerFaceColor','k')
%         plot(Zheli(end,2),Zheli(end,1),'rp','MarkerSize',8)
%         plot(50,0,'bx','MarkerSize',8,'MarkerFaceColor','b')
%         plot(0,50,'bx','MarkerSize',8,'MarkerFaceColor','b')
%         plot(50,50,'bx','MarkerSize',8,'MarkerFaceColor','b')
%         plot(Zheli(:,2),Zheli(:,1),'k-','LineWidth',2)
%         xlabel('y (ft.)')
%         ylabel('x (ft.)')
%         zlabel('z (ft.)')
%         axis equal
%         legend('Initial Point','Final Point','Waypoints','location','best')
%         if PLOTQUADDOTSTATES
%             Plot6([1:12],{t},Zhelidot,1,'ft',0,'',[],[],'Quadrotor Dot');
%         end
    end
end
if perrors
    all = dlmread([root1,'Errors.OUT']);
    terror = all(:,1);
    tether_yaw = all(:,2);
    tether_yaw_pure = all(:,3);
    bird_yaw = all(:,4);
    bird_yaw_pure = all(:,5);
    plottool(1,'Errors',18,'Time (sec)','Tether Yaw Angle (deg)');
    plot(terror,tether_yaw.*180/pi,'k-','LineWidth',2)
    plot(terror,tether_yaw_pure*180/pi,'k--','LineWidth',2)
    legend('Polluted','Pure')
    
    plottool(1,'Bird Yaw',18,'Time (sec)','Bird Yaw Angle (deg)');
    plot(terror,bird_yaw*180/pi,'k-','LineWidth',2)
    plot(terror,bird_yaw_pure*180/pi,'k--','LineWidth',2)
    legend('Polluted','Pure')
    
    plottool(1,'Feedback Signal',18,'Time (sec)','Bird Yaw Angle - Tether Yaw Angle (deg)')
    plot(terror,(bird_yaw-tether_yaw)*180/pi,'k-','LineWidth',2)
    plot(terror,(bird_yaw_pure-tether_yaw_pure)*180/pi,'k--','LineWidth',2)
    legend('Polluted','Pure')
end

if pcontrols
    all = dlmread([root1,'Controls.OUT']);
    time = all(:,1);
    Names = {'Aileron','Elevator','Rudder','Flaps'};
    YAXES = {'Aileron (deg)','Elevator (deg)','Rudder (deg)','Flaps (deg)'};
    for idx = 1:4
        plottool(1,Names{idx},18,'Time (sec)',YAXES{idx});
        plot(time,all(:,idx+1)*180/pi)
    end
end

if ptowed
    Ztowed = Z(:,1:13);
    if COMPARE
        Zaother = Zother(:,1:13);
        Plot6Quat([3 40],{t,tOther},Ztowed,1,'ft',{Zaother},LegendNames,'Towed');
    else
        Plot6Quat([1 2 3 8:13 40],{t},Ztowed,1,'ft',[],'','Towed');
        
        %%%Plot Angle of Attack
        uactual = Ztowed(:,8);
        vactual = Ztowed(:,9);
        wactual = Ztowed(:,10);
        alfaA = atan2(wactual,uactual).*180/pi;
        betaA = atan2(vactual,uactual).*180/pi;
        plottool(1,'Alfa',18,'Time (sec)','Angle of Attack (deg)')
        plot(t,alfaA,'k-','LineWidth',2)
        
        plottool(1,'Beta',18,'Time (sec)','Angle of Sideslip (deg)')
        plot(t,betaA,'k-','LineWidth',2)
        
        %%%Compute Standard Deviation
        ptp = quat2euler(Ztowed(:,4:7),0,t,'Towed');
        xyz = Ztowed(:,1:3);
        [r,c] = size(xyz);
        half = floor(r/2);
        averageptp = mean(ptp(half:end,:));
        Zheli = Z(:,14:25);
        xyzheli = Zheli(:,1:3);
        xyz(:,1) = xyz(:,1)-xyzheli(:,1);
        xyz(:,3) = xyz(:,3)-xyzheli(:,3);
        averagexyz = mean(xyz(half:end,:));
        format long g
        disp('Mean X Y Z (ft) and Phi theta psi (deg) =  ')
        disp([averagexyz averageptp*180/pi]')
        
        stdxyz = std(xyz(half:end,:));
        stdptp = std(ptp(half:end,:));
        disp('Std Dev X Y Z (ft) and Phi theta psi (deg) =  ')
        disp([stdxyz stdptp*180/pi]')
        disp(norm(stdptp)*180/pi)
    end
end

if ptether
    dim1 = 'ft';
    Names = {'X','Y','Z','U','V','W'};
    colors = {'b','g','r','y','m','c'};
    linestyle = {'-','--'};
    ylabels = {['x beads (',dim1,')'],['y beads (',dim1,')'],['z beads (',dim1,')'],['u(',dim1,'/s)'],['v(',dim1,'/s)'],['w(',dim1,'/s)']};
    %%%This plots the states of each tether point
    for ii = 1:3 %%%
        plottool(1,Names{ii},18,'Time (s)',ylabels{ii});
        %for jj = 1:nT:nT
        ctr = 0;
        for jj = 1:nT
            ctr = ctr + 1;
            if ctr > length(colors)
                ctr = 1;
            end
            s = (jj-1)*6;
            plot(t,Z(:,26+(ii-1)+s),[colors{ctr},linestyle{1}],'LineWidth',2)
            if COMPARE
                plot(tOther,Zother(:,26+(ii-1)+s),[colors{ctr},linestyle{2}],'LineWidth',3)
            end
        end
    end
    
    Tether_Length = 0*t;
    Zheli = Z(:,14:25);
    Ztowed = Z(:,1:13);

    if nT > 0
        for tdx = 1:length(t)
            Zrow = Z(tdx,:);
            ptp_heli = Zrow(17:19);
            rHFB = [0;0;WLCG_HELI];     %%% REVISIT The file was readng in grav, aero, and contact flag from Quad.COPTER file, which is incorrect
                                        %%% So I manually hardcoded this for now until we
                                        %%% can fix the getTetherConnectionPointHeli file
            T = R123(ptp_heli(1),ptp_heli(2),ptp_heli(3));
            rHI = Zrow(14:16)';
            rF = rHI + T*rHFB;
            delx = Z(tdx,26)-rF(1); %%This is the first bead
            dely = Z(tdx,27)-rF(2);
            delz = Z(tdx,28)-rF(3);
%             fprintf('First Connection Point %f \n',sqrt(delx^2+dely^2+delz^2))
            Tether_Length(tdx) = sqrt(delx^2+dely^2+delz^2);
%             if tdx > 500
%             tdx
%             rF'
%             Z(tdx,26:26+65)
%             disp('-------------------------')
%             Ztowed(tdx,1:3)
%             end
            
            for ndx = 1:nT-2        % REVISIT : Changed this to nT - 2 because there are nT +1 elements
                                    % The first element is the r(bead=1) - rcg of the driver
                                    % Then there are nT - 2 elements in between
                                    % Then the final element is rcg of towed body -
                                    % r(bead=n)
                s = (ndx-1)*6;
                %%Compute the difference between tether ndx and ndx+1
                delx = -Z(tdx,26+s)+Z(tdx,26+s+6);
                dely = -Z(tdx,27+s)+Z(tdx,27+s+6);
                delz = -Z(tdx,28+s)+Z(tdx,28+s+6);
%                 ndx
%                 previous = [Z(tdx,26+s) , Z(tdx,27+s) , Z(tdx,28+s)]
%                 nextone  = [Z(tdx,26+s+6) , Z(tdx,27+s+6) , Z(tdx,28+s+6)]
                delxyz = sqrt(delx^2+dely^2+delz^2);
                Tether_Length(tdx) = Tether_Length(tdx) + delxyz;
            end
            s = (nT-2)*6;       % Changed this as well to match the loop's structure
%             Ztowed(tdx,1:3)
            delx = -Z(tdx,26+s)+Ztowed(tdx,1); %%%This is the last bead
            dely = -Z(tdx,27+s)+Ztowed(tdx,2);
            delz = -Z(tdx,28+s)+Ztowed(tdx,3);
%             fprintf('Last Connection Point %f \n',sqrt(delx^2+dely^2+delz^2))
%             disp('----------------------------------------------------------')
            Tether_Length(tdx) = Tether_Length(tdx) + sqrt(delx^2+dely^2+delz^2);
        end
    else
        Tether_Length = mat_norm(Zheli(:,1:3)-Ztowed(:,1:3));
    end
%     [h2,f2] = plottool(1,'Tether Length',18,'Time(sec)','Change in Tether Length (ft)');
%     %Tether_Length_inches = Tether_Length*12;
    l0 = 10 - WLCG;         % Added WLCG factor to add length to the tether
    Delta_Tether_Length = Tether_Length - l0;
    Percent_Change_Tether = ((Delta_Tether_Length/l0) * 100);
%     plot(t,Delta_Tether_Length,'k-','LineWidth',2)
    
    plottool(1,'Change in Tether Length',18,'Time (s)','Elongation (%)');
    plot(t,Percent_Change_Tether,'k-')
%     grid off
    
    plottool(1,'Tension',18,'Time (s)','Tension (lbf)');
    
%     grid off
    ctr = 0;
    for ii = [1]
        tension = Z(:,26 + 5 + (nT-1)*6 + ii);
        tplot = t;
        ctr = ctr + 1;
        if ctr > length(colors)
            ctr = 1;
        end
        plot(tplot,tension)%,[colors{ctr},linestyle{1}],'LineWidth',2)

        if COMPARE
            tension = Zother(:,26+5+(nT-1)*6 + ii);
            %tOtherplot = tOther;
            %tOtherplot(tension < 0) = [];
            %tension(tension < 0) = [];
            %if ii == 1
            plot(tOther,tension,[colors{ctr},linestyle{2}],'LineWidth',2)
            %end
            legend(LegendNames)
        end
    end
    if ~COMPARE
        %%% REVISIT REVISIT
%         legend('Tension at Helicopter','Tension at Towed Body') %% 1 = Tension at Helicopter, %nT+1 = Tension at Towed Body
    end
%     plottool(1,'Tension Dot',18,'Time(sec)','Tension Dot (N/s)');
%     for ii = 1:(nT+1)
% %         tensiondot = Z(:,114 + 44 + 5 + (nT-1)*6 + ii );
%         tensiondot = Z(:,92 + ii );
%         tplot = t;
%         tplot(tensiondot < 0) = [];
%         tensiondot(tensiondot < 0) = [];
%         plot(tplot,tensiondot,colors{ii},'LineWidth',2)
%         if COMPARE
%             tension = Zother(:,44+5+(nT-1)*6 + ii);
%             tOtherplot = tOther;
%             tOtherplot(tension < 0) = [];
%             tension(tension < 0) = [];
%             plot(tOtherplot,tension,colors{ii+1},'LineWidth',2)
%             legend(LegendNames)
%         end
%     end
end

if ptowed
% % % %     %%% REVISIT COMMENTED THIS OUT BECAUSE IT WAS GETTING ANNOYING
% % % %     Zheli = Z(:,14:25);
% % % %     
% % % %     %%%Plot a top down trajectory
% % % %     plottool(1,'Absolute Traj',18,'Y (ft)','X (ft)')
% % % %     plot(Zheli(:,2)-Zheli(1,2),Zheli(:,1)-Zheli(1,1),'k-','LineWidth',2)
% % % %     plot(Ztowed(:,2)-Zheli(1,2),Ztowed(:,1)-Zheli(1,1),'k--','LineWidth',2)
% % % % 
% % % % %     plot(Zheli(:,1)-Zheli(1,1),Zheli(:,2)-Zheli(1,2),'k-','LineWidth',2)
% % % % %     plot(Ztowed(:,1)-Zheli(1,1),Ztowed(:,2)-Zheli(1,2),'k--','LineWidth',2)
% % % % %     plot(50,0,'bx','MarkerSize',8,'MarkerFaceColor','b')
% % % % %     plot(0,50,'bx','MarkerSize',8,'MarkerFaceColor','b')
% % % % %     plot(50,50,'bx','MarkerSize',8,'MarkerFaceColor','b')
% % % %     legend('Quadrotor Trajectory','Towed Body Trajectory')%,'Waypoints')
% % % % %     reverse('y')
% % % % %     axis equal
% % % % 
% % % %     %%%Compute Tether angle
% % % %     time = t;
% % % %     xtowed = Ztowed(:,1);
% % % %     ytowed = Ztowed(:,2);
% % % %     reelx = Zheli(:,1);
% % % %     deltax1 = reelx(1) - xtowed(1);
% % % %     deltax = (reelx-xtowed);
% % % %     deltay = ytowed(1)-ytowed+BLCG_HELI;
% % % %     plottool(1,'Heli Frame',18,'Time (sec)','Delta X (ft)');
% % % %     plot(time,deltax-deltax1,'k-','LineWidth',2)
% % % %     
% % % %     plottool(1,'Heli Frame',18,'Crossrange (ft)','Delta X (ft)');
% % % %     plot(deltay,deltax-deltax1,'k-','LineWidth',2)
% % % %     
% % % %     %%%Plot a CEP Curve
% % % %     xmean = mean(deltax-deltax1);
% % % %     ymean = mean(deltay);
% % % %     radius = sort(sqrt((deltax-deltax1-xmean).^2+(deltay-ymean).^2));
% % % %     CEP = radius(round(length(radius)/2));
% % % %     disp('Mean X,Y (ft) and CEP')
% % % %     disp([xmean ymean CEP]')
% % % %     theta = linspace(0,2*pi,100);
% % % %     xcircle = CEP*cos(theta)+xmean;
% % % %     ycircle = CEP*sin(theta)+ymean;
% % % %     plot(ycircle,xcircle,'b-','LineWidth',2)
% % % %     CubeDraw(0.3772966,0.656168,1,0,0,0,0,0,0,[1 0 0])
% % % %     legend('Towed System Trajectory','CEP')%,'Nominal Position')
% % % %     axis equal
% % % %     
% % % %     reelz = Zheli(:,3);
% % % %     ztowed = Ztowed(:,3);
% % % %     deltaz = reelz-ztowed;
% % % %     Tether_Angle = atan2(deltax,deltaz)*180/pi;
% % % %     tfilter = time;
% % % %     [h1,f1] = plottool(1,'Tether Vertical Angle',18,'Time(sec)','Tether Vertical Angle(deg)');
% % % %     plot(tfilter,Tether_Angle,'k-','LineWidth',2)
% % % %     [h2,f2] = plottool(1,'Tether Horizontal Angle',18,'Time(sec)','Tether Horizontal Angle(deg)');
% % % %     Tether_H_Angle = arcfun(deltay,deltax)*180/pi;
% % % %     plot(time,Tether_H_Angle,'k-','LineWidth',2)
% % % %     if COMPARE
% % % %       	Zheli = Zother(:,14:25);
% % % %         Zpother = Zother(:,1:13);
% % % %         time = tOther;
% % % %         reelx = Zheli(:,1);
% % % %         reely = Zheli(:,2);
% % % %         reelz = Zheli(:,3);
% % % %         psis = Zheli(:,6);
% % % %         xtowed = Zpother(:,1);
% % % %         ytowed = Zpother(:,2);
% % % %         deltax = reelx-xtowed;
% % % %         deltay = reely-ytowed;
% % % %         deltax_S = deltax.*cos(psis) + deltay.*sin(psis)-70.27;
% % % %         deltay_S = -deltax.*sin(psis) + deltay.*cos(psis);
% % % %         reelz = Zheli(:,3)+5.48;	
% % % %         ztowed = Zpother(:,3);	
% % % %         deltaz = reelz-ztowed;
% % % %         Tether_Angle = atan2(deltax_S,deltaz)*180/pi;
% % % %         plot(f1,time,Tether_Angle,'k--','LineWidth',2)
% % % %         legend(f1,LegendNames)
% % % %         Tether_H_Angle = arcfun(deltay_S,deltax_S)*180/pi;
% % % %         plot(f2,time,Tether_H_Angle,'k--','LineWidth',2)
% % % %         legend(f2,LegendNames)
% % % %     end
end
if pwind
     winds = dlmread('Output_Files/Misc.OUT');
     time = winds(:,1);
     windnames = {'Vx (ft/s)','Vy (ft/s)','Vz (ft/s)'};
     plottool(1,'Tow Winds',18,'Time (s)','WindSpeed (ft/s)');
     plot(time,winds(:,2:4),'LineWidth',2)
     legend(windnames)

%      plottool(1,'WRF Winds',18,'Time (s)','WindSpeed (ft/s)');
%      plot(time,winds(:,5:7),'LineWidth',2)
%      legend(windnames)
%      
%      plottool(1,'Dryden Winds',18,'Time (s)','WindSpeed (ft/s)');
%      plot(time,winds(:,8:10),'LineWidth',2)
%      legend(windnames)
end

if saveplots
    system('rm Frames/States*.jpg');
    system('rm Frames/States*.pdf');
    system('rm Frames/States*.png');
    system('rm Frames/States*.eps');
   ii = 50;
   while ~isempty(get(0,'CurrentFigure') )
       f = getfilename(ii,5);
       saveas(gcf,['Frames/States/Frame_',f,'.eps'])
       close(gcf)
       ii = ii - 1;
   end
   disp('Converting All Pdfs to State.pdf')
   system('convert Frames/States/*.pdf State.pdf')
end

%%%%Plot R AND F
% if pheli && ptowed
%     Rz = Zheli(:,3);
%     Rf = Ztowed(:,3);
%     plottool(1,'Reel and Connection Point',14,'Time (sec)','Change in Position (m)')
%     plot(time-50,Rz,'k-','LineWidth',2)
%     plot(time-50,Rf-49.63,'k--','LineWidth',2)
%     legend('Tow Vehicle','Towed Body')
% end