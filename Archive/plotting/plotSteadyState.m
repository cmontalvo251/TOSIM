function plotSteadyState(color,xyzh)

%%%%IMPORT DATA
[tZ] = dlmread('Output_Files/State.OUT');
tMisc = dlmread('Output_Files/Misc.OUT');
nBeads = tMisc(:,5);

parameters

t = tZ(:,1);
Z = tZ(:,2:end);
[r,c] = size(Z);
%nT = round((c/2-13-12)/7);

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
    system('rm Frames/*.jpg');
end

if vtether || vtowed
    %%%%Get SLCG
    [SLCG,WLCG,BLCG] = getTetherConnectionPoint();
    [SLCG_HELI,WLCG_HELI,BLCG_HELI] = getTetherConnectionPointHeli();
end


istart = length(t);
skip = 1;

prev_state = [0,0,0];

for ii = istart:skip:length(t)
    %pause
    hold off
    Zrow = Z(ii,:);
    offsetx = Zrow(14);
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
        ybox = 0.995/2;
        xbox = 1.583; 
        zbox = 0.1;
        CubeDraw(xbox,ybox,zbox,state(1)-offsetx,state(2),state(3),state(4),state(5),state(6),[1 1 0])
%         CubeDraw(4.0,1,1,state(1)-offsetx,state(2),state(3),state(4),state(5),state(6),[1 1 0])
        %plot3(state(1),state(2),state(3),'bo','MarkerSize',20)
        %ball(state(1),state(2),state(3),1)
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
        %%%%Compute First tether point
%         rHFB = [0;0;0]; %%% Revisit hardcoded this because the getTether file wasn't
                        %%% reading in the connection point properly
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
        rCFB = [SLCG;BLCG;WLCG];
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
            plot3(xi,yi,zi,'ms','LineWidth',3,'MarkerSize',5);
            %CubeDraw(1,1,1,xi,yi,zi,0,0,0,[0 1 0])
            hold on
        end
        plot3(xtether(end),ytether(end),ztether(end),'k*','MarkerSize',5);

        %xtether(1) = helix-offsetx;
        %ytether(1) = heliy;
        %ztether(1) = heliz;
        plot3(xtether,ytether,ztether,color,'LineWidth',2,'MarkerSize',5)
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
        if STLPLOT
            scale = 7.818*[1;1;1];
            state = Zrow(14:19);
            state(1) = state(1)-offsetx;
            fcolor = [0.5 0.5 0.4];
            draw_heli_fancy(state,xyzh,scale,fcolor);
        else 
            DrawHeli(Zrow(14)-offsetx,Zrow(15),Zrow(16),Zrow(17),Zrow(18),Zrow(19),xtether(1),ytether(1),ztether(1))
        end
    end
    %%%%
    %t(ii)
    title(['t=',num2str(t(ii)),' sec V = ',num2str(Zrow(8)),' ft/s'])
    %text(Zrow(19),Zrow(20),Zrow(21)+5,['t=',num2str(rounddigits(t(ii),1))])
    hold on
    reverse(['y','z'])
    xlabel('X (ft)')
    ylabel('Y (ft)')
    zlabel('Z (ft)')
    deltax = 450;
    deltay = 100;
    if vheli
        %zlim([-600 50])
        %xlim([shipx-deltax shipx])
        %ylim([shipy-deltay shipy+deltay])
    end
    %axis equal
    if Az+ El == 0
        if min(Z(:,1)-Z(:,14)) < -200
            xleft = min(Z(:,1)-Z(:,14));
        else
            xleft = -200;
        end
        xlim([xleft 20])
        ylim([-10 30])
        zlim([-20 200])
    else
        axis equal
    end
    axis on
    grid on
    view(Az,El)
    %view(Az+2*t(ii),El)
    %
    drawnow
    %pause
    if SAVEMOVIE
        f = getfilename(ii,5);
        saveas(gcf,['Frames/Frame_',f,'.jpg'])
    end
    %end
end
if SAVEMOVIE
    system('cd Frames/; makemovie 20 ~/Desktop/Movie.avi jpg');
    system('rm Frames/*.jpg');
end

