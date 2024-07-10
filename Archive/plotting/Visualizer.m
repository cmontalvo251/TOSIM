clear
close all
%%Let's not run purge because we don't want to run clc

%%% Notes: 
%%% - Added drawing of cube
%%% - Reversed z axis for visualization purposes

parameters

if STLPLOT
  [xh,yh,zh]=stlread('helicopter.stl');
  xyzh = [xh;yh;zh];
end

%%%%IMPORT DATA
if ~exist('tZ','var')
    [tZ] = dlmread(file1);
    %tMisc = dlmread('Figures/Runs_1_5/Helicopter_Noise/Taylor_Aero/Output_Files/Misc222.2222.OUT');
    tMisc = dlmread('Output_Files/Misc.OUT');
    nBeads = tMisc(:,5);
    if COMPARE
        tZOther = dlmread(file2);
    end
end

t = tZ(:,1);
Z = tZ(:,2:end);
[r,c] = size(Z);
%nT = round((c/2-13-12)/7)

if COMPARE
    tOther = tZOther(:,1);
    ZOther = tZOther(:,2:end);
    [rC,cC] = size(ZOther);
    if rC ~= r || cC ~= c
        COMPARE = 0;
        disp('Files do not match')
    end
end

%%%This code assumes the following
%Z = [towed(13),heli(12),tether(6*nT)]

if SAVEMOVIE
    system('rm Frames/Visualizer/*.jpg')
    system('rm Frames/Visualizer/*.png')
    system('rm Frames/Visualizer/*.eps')
    
    % Name this file whatever you want
    vidObj = VideoWriter('TOSIM_Visual.avi');
    open(vidObj);
end

if vtether || vtowed
    %%%%Get SLCG
    [SLCG,WLCG,BLCG] = getTetherConnectionPoint();
    [SLCG_HELI,WLCG_HELI,BLCG_HELI] = getTetherConnectionPointHeli();
end

plottool(1,'Visualization',14,'x','y','z');

tstart = 0;
istart = find(t >= tstart,1);

prev_state = [0,0,0];



for ii = istart:skip:length(t)
    %pause
    t(ii)
    hold off
    cla;
    Zrow = Z(ii,:);
    offsetx = Zrow(14)*0;
    new_state = Zrow(1:3);
    %if norm(new_state-prev_state) > 4.0
    prev_state = new_state;
    if COMPARE
        Zrow2 = ZOther(ii,:);
    end
    if vtowed
        %%%Towed system
        ptp = quat2euler(Zrow(4:7));
        state = [Zrow(1:3),ptp];
        %draw_plane_fancy(state,xyzf,xyzL,xyzR,xyzp,30,0)
        ybox = 0.995;
        xbox = 1.583; 
        zbox = 0.1;
        CubeDraw(xbox,ybox,zbox,state(1)-offsetx,state(2),state(3),state(4),state(5),state(6),[1 1 0])
        %plot3(state(1),state(2),state(3),'bo','MarkerSize',20)
        %ball(state(1)-offsetx,state(2),state(3),1)
        hold on
    end
    helix = Zrow(14);
    heliy = Zrow(15);
    heliz = Zrow(16);
    if vtether
        %%%Tether(1st point is at 44)
        nT = nBeads(ii);
        xtether = zeros(nT+2,1);
        ytether = zeros(nT+2,1);
        ztether = zeros(nT+2,1);
        %%%%Compute First tether point, connection point on platform
%         rHFB = [0 ; 0; WLCG_HELI];      %%% REVISIT Hardcoded this because the function isn't reading the file properly

        rHFB = [SLCG_HELI;BLCG_HELI;WLCG_HELI];
        ptp_heli = Zrow(17:19);
        T = R123(ptp_heli(1),ptp_heli(2),ptp_heli(3));
        rHI = Zrow(14:16)';
        rF = rHI + T*rHFB;
        
        xtether(1) = rF(1)-offsetx;
        ytether(1) = rF(2);
        ztether(1) = rF(3);
        %%%Compute last tether point
        rCI = Zrow(1:3)';
        %%% Connection point on payload
        rCFB = [SLCG;BLCG;WLCG]; %%% REVISIT Changed WLCG to 0 for visuals. - N.H. 5/2/17
        ptp = quat2euler(Zrow(4:7));
        T = R123(ptp(1),ptp(2),ptp(3));
        rF = rCI + T*rCFB;
        xtether(end) = rF(1)-offsetx; 
        ytether(end) = rF(2); 
        ztether(end) = rF(3);
        for jj = 1:nT
            s = (jj-1)*6;
            xi = Zrow(26+s)-offsetx;
            yi = Zrow(27+s);
            zi = Zrow(28+s);
            xtether(jj+1) = xi;
            ytether(jj+1) = yi;
            ztether(jj+1) = zi;
            %mesh(Xp+xi,Yp+yi,Zp+zi)
            plot3(xi,yi,zi,'mo','LineWidth',3,'MarkerSize',5);
            %CubeDraw(1,1,1,xi,yi,zi,0,0,0,[0 1 0])
            hold on
        end
        plot3(xtether,ytether,ztether,'k-','LineWidth',1,'MarkerSize',5)
         if COMPARE
             for jj = 1:nT
                 s = (jj-1)*6;
                 xi = Zrow2(26+s)-offsetx;
                 yi = Zrow2(27+s);
                 zi = Zrow2(28+s);
                 %mesh(Xp+xi,Yp+yi,Zp+zi)
                 plot3(xi,yi,zi,'cs','LineWidth',3,'MarkerSize',10);
                 %CubeDraw(1,1,1,xi,yi,zi,0,0,0,[0 1 0])
             end
         end
    end
    if vheli
        %%%Heli
        dx = 3;
        dy = 2;
        dz = 0.25;
        if STLPLOT
            scale = 7.818*[1;1;1];
            state = Zrow(14:19);
            state(1) = state(1)-offsetx;
            fcolor = [0.5 0.5 0.4];
            draw_heli_fancy(state,xyzh,scale,fcolor);
        else 
%             CubeDraw(dx,dy,dz,Zrow(14)-offsetx,Zrow(15),-Zrow(16),Zrow(17),Zrow(18),Zrow(19),[0 0 1]);
            
              %%% REVISIT REVISIT UNCOMMENT TO PLOT THE QUADCOPTER
            DrawHeli(Zrow(14)-offsetx,Zrow(15),Zrow(16),Zrow(17),Zrow(18),Zrow(19),xtether(1),ytether(1),ztether(1))
        end
        %%%Plot connection point
        %plot3(-33,0,-10,'ms','LineWidth',3','MarkerSize',10);
        hold on
    end
    %%%%
    t(ii);
    %title(['t=',num2str(t(ii)),' sec V = ',num2str(Zrow(20)),' ft/s'])
    %text(Zrow(19),Zrow(20),Zrow(21)+5,['t=',num2str(rounddigits(t(ii),1))])
    hold on
    reverse(['y','z'])        %%% REVISIT
%     reverse(['x','y'])
    xlabel('x (ft)')
    ylabel('y (ft)')
    zlabel('z (ft)')
    deltax = 450;
    deltay = 100;
    if vheli
        %zlim([-600 50])
        %xlim([shipx-deltax shipx])
        %ylim([shipy-deltay shipy+deltay])
    end
    %axis equal
    if Az+ El == 0
        xlim([-200 20])
        ylim([-10 30])
        zlim([-20 200])
    else
        axis equal
    end
    axis on
%     xlim([-1 1])
%     ylim([0 90])
% % %     zlim([9 20])
%     zlim([8 20])
    grid off
%    view(0,90)       %%% Top-down view
    view(Az,El)       %%% Isometric view
%    view(Az+2*t(ii),El)    %%% Rotation isometric view
    light('Position',-1000.*[0 0 2]);
%     pause
%     pause
    drawnow
    
    
    %%% REVISIT: This way works for me. - Nemo
    if SAVEMOVIE
        myframe = getframe(gcf);
        writeVideo(vidObj,myframe);
    end
%     if SAVEMOVIE
%         f = getfilename(ii,5);
%         saveas(gcf,['Frames/Visualizer/Frame_',f,'.png'])
%     end
    %end
end
% if SAVEMOVIE
%     system('cd Frames/; makemovie 30 ~/Desktop/Movie.avi jpg');
%     %system('rm Frames/*.jpg');
% end

if SAVEMOVIE == 1
    close(vidObj);
end




