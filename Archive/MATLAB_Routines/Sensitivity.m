function Sensitivity(var1in,savetime,xaxis)
STDFILE = 'Output_Files/STD.OUT';
MEANFILE = 'Output_Files/MEAN.OUT';
!rm Output_Files/STD.OUT
!rm Output_Files/MEAN.OUT
if ~exist('var1in','var')
    var1in = realmax;
    savetime = 50;
    xaxis = '';
end

[h1,f1] = plottool(1,'Phi',18,'Time (sec)','\phi (deg)');
[hf,ff] = plottool(1,'Forces Towed Body',18,'Time (sec)','Connection Point Forces (lbf)');

%plottool(1,'Heli Frame',18,'Time (sec)','Delta X From Nominal (ft)');
color = {'b-','r-','g-','c-','k-'};
ctr = 1;
for var1 = var1in
    if var1~=realmax
        file1 = ['Output_Files/State',num2str(var1),'.OUT']
        try 
            file2 = ['Output_Files/Misc',num2str(var1),'.OUT'];
        catch me
            file2 = [];
        end
    else
        file1 = ['Output_Files/State.OUT']
    end
    [tZ] = dlmread(file1);
    s = find(tZ(:,1)>savetime,1);
    t = tZ(s:end,1);
    
    Z = tZ(s:end,2:end);
    Ztowed = Z(:,1:13);
    ptp = quat2euler(Ztowed(:,4:7),0,t,'Towed');
    plot(f1,t-savetime,(ptp(:,1)*180/pi),color{ctr},'LineWidth',2)
    xyz = Ztowed(:,1:3);
    
    if isempty(file2) && 0 %%%IF YOU WANT TO PLOT THE STUFF BELOW CHANGE TO 1
        data = dlmread(file2);
        time = data(s:end,1);
        FXYZ = data(s:end,6:8);
        %MXYZ = data(:,9:11);
        plot(ff,time,matnorm(FXYZ),color{ctr},'LineWidth',2)
        fig = figure();
        [AX,H1,H2] = plotyy(time,abs(ptp(:,1)*180/pi),time,matnorm(FXYZ));
        hold on
        h3 = plot(AX(1),time,xyz(:,2));
        set(H1,'LineStyle','--')
        title(['Stationline = ',num2str(var1),' ft'])
        set(get(AX(1),'Ylabel'),'String','|\phi| (deg) (Dashed) and Y-coordinate (ft) (Solid)')
        set(get(AX(2),'Ylabel'),'String','Tension (lbf)')
        xlabel('Time (sec)')
    end
    

    averageptp = mean(ptp);
    Zheli = Z(:,14:25);
    
    %var1 = var1*10.5; %%%REVISIT REVISIT REVISIT ONLY FOR WINDSPEED!!!!!
    %vHeli = Zheli(:,8); %%%REVISIT REVISIT REVISIT ONLY FOR HELI NOISE!!!!
    %var1 = std(vHeli);
    
    xyzheli = Zheli(:,1:3);
    xyz(:,1) = xyz(:,1)-xyzheli(:,1);
    xyz(:,3) = xyz(:,3)-xyzheli(:,3);
    %plot(xyz(:,2))
    %pause
    averagexyz = mean(xyz); 
    %format long g
    disp('Mean X Y Z (ft) and Phi theta psi (deg) =  ')
    disp([averagexyz,averageptp*180/pi])
    %fprintf(meanfid,'%lf \n',[var,averagexyz averageptp*180/pi]');
    dlmwrite(MEANFILE,[var1,averagexyz,averageptp*180/pi],'delimiter',' ','-append');
    stdxyz = std(xyz);
    stdptp = std(ptp);
    disp('Std Dev X Y Z (ft) and Phi theta psi (deg) =  ')
    disp([stdxyz,stdptp*180/pi])
    %fprintf(stdfid,'%lf \n',[var,stdxyz stdptp*180/pi]')
    dlmwrite(STDFILE,[var1,stdxyz,stdptp*180/pi],'delimiter',' ','-append');
    
%     time = tZ(:,1);
%     [SLCG_HELI,WLCG_HELI,BLCG_HELI] = getTetherConnectionPointHeli();
%     Z = tZ(:,2:end);
%     Zheli = Z(:,14:25);
%     Ztowed = Z(:,1:13);
%     xtowed = Ztowed(:,1);
%     ytowed = Ztowed(:,2);
%     reelx = Zheli(:,1);
%     deltax1 = reelx(1) - xtowed(1);
%     deltax = (reelx-xtowed);
%     deltay = ytowed(1)-ytowed+BLCG_HELI;
%     dxnominal = deltax-deltax1-mean(deltax-deltax1);
%     %plot(time,dxnominal,color{ctr},'LineWidth',2)
%     order = 4;
%     samplingfrequency = 1/(time(2)-time(1));
%     pburg(dxnominal,order,[],samplingfrequency);
%     changeCurrentColor(color{ctr})
    ctr = ctr+1;
    if ctr > length(color)
        ctr = 1;
    end

    %disp('FFT')
    %fft(dxnominal)
    
end

legend(f1,vec2cell(var1in))
legend(vec2cell(var1in))

STDIN = dlmread(STDFILE);
MEANIN = dlmread(MEANFILE);

var1 = STDIN(:,1);
stdxyz = STDIN(:,2:4);
stdptp = STDIN(:,5:7);

plottool(1,'STDXYZ',12,xaxis,'Standard Deviation XYZ (ft)')
plot(var1,stdxyz,'-*','LineWidth',2)
legend('X','Y','Z')
%xlim([0 20./12])
plottool(1,'STDPTP',12,xaxis,'Standard Deviation PTP (deg)')
plot(var1,stdptp,'-*','LineWidth',2)
legend('\phi','\theta','\psi')
%xlim([0 20./12])

var1 = MEANIN(:,1);
meanxyz = MEANIN(:,2:4);
meanptp = MEANIN(:,5:7);

plottool(1,'MEANXYZ',12,xaxis,'Mean XYZ (ft)')
plot(var1,meanxyz,'-*','LineWidth',2)
legend('X','Y','Z')

plottool(1,'MEANPTP',12,xaxis,'Mean PTP (deg)')
plot(var1,meanptp,'-*','LineWidth',2)
legend('\phi','\theta','\psi')

pause

while ~isempty(get(0,'CurrentFigure') )
    saveas(gcf,['Frames/',get(gcf,'Name'),'.fig'])
    saveas(gcf,['Frames/',get(gcf,'Name'),'.eps'],'epsc')
    close(gcf)
end