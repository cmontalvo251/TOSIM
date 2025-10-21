function helifile = gethelifile()

files = importdata('Input_Files/TOMAD.ifiles');
helifile = files{2};
s = find(helifile=='I',1);
e = find(helifile=='!',1)-1;
helifile = helifile(s:e);
helifile(helifile==' ')=[];
helifile(helifile=='''')=[];
%d = double(helifile);
%l = find(d==73);
%le = l(end);
%helifile = helifile(1:le);
%%%Remove leading tabs
dbl = double(helifile);
dbl(dbl==9) = [];
helifile = char(dbl);
disp(['Reading Heli File: ',helifile])

