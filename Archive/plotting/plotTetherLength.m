function plotTetherLength(color,NominalL)
[tZ] = dlmread('Output_Files/State.OUT');
t = tZ(:,1);
Z = tZ(:,2:end);
[r,c] = size(Z);
nT = round((c/2-13-12)/7);
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
%Tether_Length_inches = Tether_Length*12;
Delta_Tether_Length = Tether_Length-NominalL;
%Percent_Change_Tether = (Delta_Tether_Length_inches/Tether_Length_inches(1))*100;
plot(t,Delta_Tether_Length,color,'LineWidth',2)
