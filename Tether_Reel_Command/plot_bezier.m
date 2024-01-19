purge

%A = dlmread('ReelCommand.THR');
A = dlmread('PayInPayOut.TCOM');
t = A(1:200);
l = A(201:400);

plottool(1,'Name',18,'Time(sec)','Unstretched Length(m)');
plot(t,l,'LineWidth',2)