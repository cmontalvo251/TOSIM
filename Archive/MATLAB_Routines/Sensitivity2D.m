function Sensitivity2D(var1in,var2in,savetime,xaxis,yaxis)
STDFILE = 'Output_Files/STD.OUT';
MEANFILE = 'Output_Files/MEAN.OUT';
!rm Output_Files/STD.OUT
!rm Output_Files/MEAN.OUT

color = {'b-','r-','g-','c-','k-'};
ctr = 1;
for var1 = var1in
    for var2 = var2in
        file1 = ['Output_Files/State',num2str(var1),num2str(var2),'.OUT']
        [tZ] = dlmread(file1);
        s = find(tZ(:,1)>savetime,1);
        t = tZ(s:end,1);

        Z = tZ(s:end,2:end);
        Ztowed = Z(:,1:13);
        ptp = quat2euler(Ztowed(:,4:7),0,t,'Towed');
        %plot(f1,t,ptp(:,1)*180/pi,color{ctr},'LineWidth',2)
        xyz = Ztowed(:,1:3);
        averageptp = mean(ptp);
        Zheli = Z(:,14:25);
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
        dlmwrite(MEANFILE,[var1,var2,averagexyz,averageptp*180/pi],'delimiter',' ','-append');
        stdxyz = std(xyz);
        stdptp = std(ptp);
        disp('Std Dev X Y Z (ft) and Phi theta psi (deg) =  ')
        disp([stdxyz,stdptp*180/pi])
        %fprintf(stdfid,'%lf \n',[var,stdxyz stdptp*180/pi]')
        dlmwrite(STDFILE,[var1,var2,stdxyz,stdptp*180/pi],'delimiter',' ','-append');
        
        ctr = ctr+1;
        if ctr > length(color)
            ctr = 1;
        end
    end        
end

%legend(f1,vec2cell(var1in))
%legend(vec2cell(var1in))

STDIN = dlmread(STDFILE);
MEANIN = dlmread(MEANFILE);

[h1,f1] = plottool(1,'STD1',12,xaxis,yaxis,'Standard Deviation XYZ (ft)');
[h2,f2] = plottool(1,'STD2',12,xaxis,yaxis,'Standard Deviation PTP (deg)');
[h3,f3] = plottool(1,'Mean',12,xaxis,yaxis,'Mean XYZ (ft)');
[h4,f4] = plottool(1,'Mean',12,xaxis,yaxis,'Mean PTP (deg)');
[vv2,vv1] = meshgrid(var2in,var1in);
stdxyz = zeros(length(var1in),length(var2in),3);
stdptp = zeros(length(var1in),length(var2in),3);
meanxyz = zeros(length(var1in),length(var2in),3);
meanptp = zeros(length(var1in),length(var2in),3);
ctr = 0;
for idx = 1:length(var1in)
    for jdx = 1:length(var2in)
        ctr = ctr + 1;
        plot3(f1,var1in(idx),var2in(jdx),STDIN(ctr,3),'b*','LineWidth',2)
        plot3(f1,var1in(idx),var2in(jdx),STDIN(ctr,4),'r*','LineWidth',2)
        plot3(f1,var1in(idx),var2in(jdx),STDIN(ctr,5),'g*','LineWidth',2)
        stdxyz(idx,jdx,1) = STDIN(ctr,3);
        stdxyz(idx,jdx,2) = STDIN(ctr,4);
        stdxyz(idx,jdx,3) = STDIN(ctr,5);
        if ctr == 1
            legend(f1,'X','Y','Z')
        end
        plot3(f2,var1in(idx),var2in(jdx),STDIN(ctr,6),'b*','LineWidth',2)
        plot3(f2,var1in(idx),var2in(jdx),STDIN(ctr,7),'r*','LineWidth',2)
        plot3(f2,var1in(idx),var2in(jdx),STDIN(ctr,8),'g*','LineWidth',2)
        stdptp(idx,jdx,1) = STDIN(ctr,6);
        stdptp(idx,jdx,2) = STDIN(ctr,7);
        stdptp(idx,jdx,3) = STDIN(ctr,8);
        if ctr == 1
            legend(f2,'\phi','\theta','\psi')
        end
        plot3(f3,var1in(idx),var2in(jdx),MEANIN(ctr,3),'b*','LineWidth',2)
        plot3(f3,var1in(idx),var2in(jdx),MEANIN(ctr,4),'r*','LineWidth',2)
        plot3(f3,var1in(idx),var2in(jdx),MEANIN(ctr,5),'g*','LineWidth',2)
        meanxyz(idx,jdx,1) = MEANIN(ctr,3);
        meanxyz(idx,jdx,2) = MEANIN(ctr,4);
        meanxyz(idx,jdx,3) = MEANIN(ctr,5);
        if ctr == 1
            legend(f3,'X','Y','Z')
        end
        plot3(f4,var1in(idx),var2in(jdx),MEANIN(ctr,6),'b*','LineWidth',2)
        plot3(f4,var1in(idx),var2in(jdx),MEANIN(ctr,7),'r*','LineWidth',2)
        plot3(f4,var1in(idx),var2in(jdx),MEANIN(ctr,8),'g*','LineWidth',2)
        meanptp(idx,jdx,1) = MEANIN(ctr,6);
        meanptp(idx,jdx,2) = MEANIN(ctr,7);
        meanptp(idx,jdx,3) = MEANIN(ctr,8);
        if ctr == 1
            legend(f4,'\phi','\theta','\psi')
        end
    end
end
% mesh(f1,vv1,vv2,stdxyz(:,:,1))
% mesh(f1,vv1,vv2,stdxyz(:,:,2))
% mesh(f1,vv1,vv2,stdxyz(:,:,3))
mesh(f2,vv1,vv2,stdptp(:,:,1))
mesh(f2,vv1,vv2,stdptp(:,:,2))
mesh(f2,vv1,vv2,stdptp(:,:,3))
% 
% mesh(f3,vv1,vv2,meanxyz(:,:,1))
% mesh(f3,vv1,vv2,meanxyz(:,:,2))
% mesh(f3,vv1,vv2,meanxyz(:,:,3))
% mesh(f4,vv1,vv2,meanptp(:,:,1))
% mesh(f4,vv1,vv2,meanptp(:,:,2))
% mesh(f4,vv1,vv2,meanptp(:,:,3))