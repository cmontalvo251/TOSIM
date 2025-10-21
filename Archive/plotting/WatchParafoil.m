purge

parameters

%%%%IMPORT DATA
if ~exist('tZ','var')
    [tZ] = dlmread(file1);
end

t = tZ(:,1);
Z = tZ(:,2:end);
[r,c] = size(Z);
nT = round((c/2-13-18-12)/7)

%%%This code assumes the following
%Z = [parafoil_states(18),aircraft(13),ship(12),tether(6*nT)]

if SAVEMOVIE
    system('rm Frames/*.jpg')
end

plottool(1,'Visualization',12,'','','');

tstart = 0;
istart = find(t > tstart,1);
prevrG = [0;0;0];
for ii = istart:skip:length(t)
    hold off
%    cla;
    Zrow = Z(ii,:);
    rG = [0;Zrow(2);Zrow(3)];
    if norm(prevrG-rG) > 100
        prevrG = rG;
        %%%%plot
        rF = DrawParafoil(Zrow(1),0.1*Zrow(2),0.05*Zrow(3),Zrow(4),Zrow(5),Zrow(6),Zrow(7),Zrow(8),Zrow(9));
        %%%%
        t(ii)
        %title(num2str(t(ii)))
        hold on
        reverse(['y','z'])
        %xlabel('x')
        %ylabel('y')
        %zlabel('z')
        axis equal
        axis on
        grid on
        view(Az,El)
        light('Position',-1000.*[-2 2 2],'Style','infinite');
        drawnow
        if vtether
            %%%Tether(1st point is at 44)
            for jj = nT
                s = (jj-1)*6;
                xi = Zrow(44+s);
                yi = Zrow(45+s);
                zi = Zrow(46+s);
                plot3([rF(1) 165/10],[rF(2) 0],[rF(3) 0],'k-','LineWidth',1,'MarkerSize',10);
                hold on
            end
        end
        if SAVEMOVIE
            f = getfilename(ii,5);
            saveas(gcf,['Frames/Frame_',f,'.jpg'])
        end
    end
end
if SAVEMOVIE
    system('cd Frames/; makemovie 20 ~/Desktop/Movie.avi jpg');
end


