clear
close all
clc

dataloc='/home/carlos/Documents/WRF_Wind_Data/Data25dx_HF=0.3/';
UVWfrontend

z = (656-42)/3.28;
z = 200;

uu = zeros(length(xcoord),length(ycoord));
vv = uu;
ww = uu;

for idx = 1:length(xcoord)
    for jdx = 1:length(ycoord)
        uvw = 3.28*uvwout(xcoord(idx),ycoord(jdx),z,0,dataloc,0);
        uu(idx,jdx) = uvw(1);
        vv(idx,jdx) = uvw(2);
        ww(idx,jdx) = uvw(3);
    end
end

mesh(uu)
title('u')
figure()
mesh(vv)
title('v')
figure()
mesh(ww)
title('w')