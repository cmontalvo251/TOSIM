%%%Tether(1st point is at 44)
parameters

close all

[tZ] = dlmread(file1);

t = tZ(:,1);
Z = tZ(:,2:end);
[r,c] = size(Z);
nT = round((c/2-13-18-12)/7)

plottool(1,'Steady State',18,'Xtether_i(m) - Xship','y','Ztether_i (m)');
Zrow = Z(end,:);
jj = 1:nT;
s = (jj-1)*6;
xi = Zrow(44+s);
yi = Zrow(45+s);
zi = Zrow(46+s);
x0 = xi(1);
%mesh(Xp+xi,Yp+yi,Zp+zi)
plot3(xi-x0,yi,-zi,'k-*','LineWidth',2);
%title(num2str(t(1)))
axis square
axis on
grid on
view(Az,El)
%%%Plot a straight line as well
xi = xi-x0;
plot3([xi(1) xi(end)],[yi(1) yi(end)],[-zi(1) -zi(end)],'k--','LineWidth',2)

if COMPARE
    [tZ] = dlmread(file2);
    t = tZ(:,1);
    Z = tZ(:,2:end);
    [r,c] = size(Z);
    nT = round((c/2-13-18-12)/7);
    Zrow = Z(end,:);
    jj = 1:nT;
    s = (jj-1)*6;
    xi = Zrow(44+s);
    yi = Zrow(45+s);
    zi = Zrow(46+s);
    x0 = xi(1);
    plot3(xi-x0,yi,zi,'k--','LineWidth',2);
end

plottool(1,'Visualization',12,'x','y','z');

tstart = 0;
istart = find(t > tstart,1);

for ii = istart:skip:length(t)
    hold off
    cla;
    Zrow = Z(ii,:);
    for jj = 1:nT
        s = (jj-1)*6;
        xi = Zrow(44+s);
        yi = Zrow(45+s);
        zi = Zrow(46+s);
        if jj == 1
            x0 = xi;
        end
        %mesh(Xp+xi,Yp+yi,Zp+zi)
        plot3(xi-x0,yi,zi,'ms','LineWidth',3,'MarkerSize',10);
        %CubeDraw(1,1,1,xi,yi,zi,0,0,0,[0 1 0])
        hold on
    end
    title(num2str(t(ii)))
    hold on
    reverse(['y','z'])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis square
    axis on
    grid on
    view(Az,El)
    light('Position',-1000.*[0 0 2],'Style','infinite');
    drawnow
end

