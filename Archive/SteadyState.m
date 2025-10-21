function SteadyState(var1in)

%time = [109.4485   95.2320   81.1476   65.5499   50.7645   36.1407];
%plottool(1,'Time to Run Per Second',12,'Number of Beads','CPU Time(sec) per Sim Time (sec)')
%plot(var1in,time./200,'b-s','LineWidth',2)

%function SteadyState(tZ)
parameters
plottool(1,'Visualization',14,'x','y','z');
%%%%Get SLCG
[SLCG,WLCG,BLCG] = getTetherConnectionPoint();
colors = {'b-','r-','g-','k-','m-','y-','c-','b-','r-','g-','m--','y:','c-'};
ctr = 0;
LSE = zeros(1,length(var1in));
for var1 = var1in
%for var1 = 30
    ctr = ctr + 1;
    if ctr > length(colors)
        ctr = 1;
    end
    file1 = ['Output_Files/Tether_Beads/State',num2str(var1),'.OUT']
    [tZ] = dlmread(file1);
    t = tZ(:,1);
    Z = tZ(:,2:end);
    [r,c] = size(Z);
    nT = round((c/2-13-12)/7);
    %%%%Limit time
    t = t(end);
    Z = Z(end,:);
    Zheli = Z(end,14:25);
    Ztowed = Z(end,1:13);
    Zrow = Z(end,:);
    offsetx = Zrow(14);
    new_state = Zrow(1:3);
    %%%Towed system
    ptp = quat2euler(Zrow(4:7));
    state = [Zrow(1:3),ptp];
    CubeDraw(4.0,1,1,state(1)-offsetx,state(2),state(3),state(4),state(5),state(6),[1 1 0])
    hold on
    dx = 20;
    dy = 10;
    dz = 20;
    CubeDraw(dx,dy,dz,Zrow(14)-offsetx,Zrow(15),Zrow(16),Zrow(17),Zrow(18),Zrow(19),[0 0 1]);
    helix = Zrow(14);
    heliy = Zrow(15);
    heliz = Zrow(16);
    xtether = zeros(nT+2,1);
    ytether = zeros(nT+2,1);
    ztether = zeros(nT+2,1);
    stether = zeros(nT+2,1);
    %%%Set the first tether point
    xtether(1) = helix-offsetx;
    ytether(1) = heliy;
    ztether(1) = heliz;
    plot3(xtether(1),ytether(1),ztether(1),[colors{ctr},'s'],'LineWidth',3,'MarkerSize',5);
    %%%Compute last tether point
    rCI = Zrow(1:3)';
    rCFB = [SLCG;BLCG;WLCG];
    ptp = quat2euler(Zrow(4:7));
    T = R123(ptp(1),ptp(2),ptp(3));
    rF = rCI + T*rCFB;
    xtether(end) = rF(1)-offsetx;
    ytether(end) = rF(2);
    ztether(end) = rF(3);
    plot3(xtether(end),ytether(end),ztether(end),[colors{ctr},'s'],'LineWidth',3,'MarkerSize',5);
    xprev = xtether(1);
    yprev = ytether(1);
    zprev = ztether(1);
    for jj = 1:nT
        s = (jj-1)*6;
        xi = Zrow(26+s)-offsetx;
        yi = Zrow(27+s);
        zi = Zrow(28+s);
        xtether(jj+1) = xi;
        ytether(jj+1) = yi;
        ztether(jj+1) = zi;
        plot3(xi,yi,zi,[colors{ctr},'s'],'LineWidth',3,'MarkerSize',5);
        hold on
        stether(jj+1) = stether(jj) + sqrt((xi-xprev)^2+(yi-yprev)^2+(zi-zprev)^2);
        xprev = xi;
        yprev = yi;
        zprev = zi;
    end
    stether(end) = stether(end-1) + sqrt((xtether(end)-xprev)^2+(ytether(end)-yprev)^2+(ztether(end)-zprev)^2);
    plot3(xtether,ytether,ztether,'k-','LineWidth',1,'MarkerSize',5)
    if var1 == 30 && find(var1in == 30)
       %%%Perform Polynomial fit
       %%%Let the independent variable be stether
       coeffX = polyfit(stether,xtether,4);
       coeffZ = polyfit(stether,ztether,4);
       xapprox = polyval(coeffX,stether);
       yapprox = ytether; %%%No need to fit y
       zapprox = polyval(coeffZ,stether);
       %plot3(xapprox,yapprox,zapprox,'k-','LineWidth',2)
    end
    if find(var1in == 30)
        %%%Compute Least Square Error based on 30 beads fit
        xfit = polyval(coeffX,stether);
        yfit = ytether;
        zfit = polyval(coeffZ,stether);
        %plot3(xfit,yfit,zfit,'ks','MarkerSize',20)
        LSE(ctr) = sum(sqrt((xfit-xtether).^2+(zfit-ztether).^2))/var1;
    end
    reverse(['y','z'])
    xlabel('X (ft)')
    ylabel('Y (ft)')
    zlabel('Z (ft)')
    if Az + El == 0
        xlim([-200 20])
        ylim([-10 30])
        zlim([-20 200])
    else
        axis equal
    end
    axis on
    grid on
    view(Az,El)
end
plottool(1,'LSE',12,'Number of Beads','Average Error Per Bead (ft)')
plot(var1in,LSE-LSE(1),'b-*','LineWidth',2)