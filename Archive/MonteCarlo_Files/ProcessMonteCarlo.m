purge

%colors = {'k-','k-.','k--','k:','k-','k-.','k--','k:','k-','k-.','k--','k:','k-'};
colors = {'b-','r-','g-','k-','m-','y-','c-','b-','r-','g-','m--','y:','c-'};
for var1 = 1:10
    [h1,f1] = plottool(1,'Tension',18,'Time(sec)','Tension (lbf)');
    title(['KV'])
    [h2,f2] = plottool(1,'Tether Length',18,'Time(sec)','Change in Tether Length(ft)');
    for var2 = 1:10
        file1 = ['Output_Files/State_',num2str(var1),'_',num2str(var2),'.OUT']
        [tZ] = dlmread(file1);
        t = tZ(:,1);
        Z = tZ(:,2:end);
        [r,c] = size(Z);
        nT = round((c/2-13-12)/7);
        %%%%Limit time
        tstart = 0;
        l = find(t >= tstart,1);
        t = t(l:end);
        Z = Z(l:end,:);
        Tether_Length = 0*t;
        Zheli = Z(:,14:25);
        Ztowed = Z(:,1:13);
        if nT > 0
            for tdx = 1:length(t)
                delx = Z(tdx,26)-Zheli(tdx,1); %%This is the first bead
                dely = Z(tdx,27)-Zheli(tdx,2);
                delz = Z(tdx,28)-Zheli(tdx,3);
                Tether_Length(tdx) = sqrt(delx^2+dely^2+delz^2);
                for ndx = 1:nT-1
                    s = (ndx-1)*6;
                    %%Compute the difference between tether ndx and ndx+1
                    delx = Z(tdx,26+s)-Z(tdx,26+s+6);
                    dely = Z(tdx,27+s)-Z(tdx,27+s+6);
                    delz = Z(tdx,28+s)-Z(tdx,28+s+6);
                    delxyz = sqrt(delx^2+dely^2+delz^2);
                    Tether_Length(tdx) = Tether_Length(tdx) + delxyz;
                end
                s = (nT-1)*6;
                delx = Z(tdx,26+s)-Ztowed(tdx,1); %%%This is the last bead
                dely = Z(tdx,27+s)-Ztowed(tdx,2);
                delz = Z(tdx,28+s)-Ztowed(tdx,3);
                Tether_Length(tdx) = Tether_Length(tdx) + sqrt(delx^2+dely^2+delz^2);
            end
        else
            Tether_Length = mat_norm(Zheli(:,1:3)-Ztowed(:,1:3));
        end
        Delta_Tether_Length = Tether_Length-196;
        plot(f2,t,Delta_Tether_Length,colors{var2},'LineWidth',2)
        for ii = [nT+1]
            tension = Z(:,26 + 5 + (nT-1)*6 + ii);
            tplot = t;
            plot(f1,tplot,tension,colors{var2},'LineWidth',2)
        end
    end
end
