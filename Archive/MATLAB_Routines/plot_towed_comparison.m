clear; close all; clc

%%% Plots uncontrolled and controlled data for towed body

load('Attitude_Controlled.mat');
load('Attitude_Uncontrolled.mat');

%%% Truncate some data
n_truncate       = 1200;

ptp_uncontrolled = ptp_uncontrolled(1:n_truncate,:);
ptp_controlled   = ptp_controlled(1:n_truncate,:);
t                = t(1:n_truncate);

ptp_uncontrolled = rad2deg(ptp_uncontrolled);
ptp_controlled   = rad2deg(ptp_controlled);


%%% Plot attitude
figure('Name','Roll')
set(gcf,'position',[550 220 800 615])
hold on
plot(t,ptp_uncontrolled(:,1))
plot(t,ptp_controlled(:,1),'r-')
xlabel('Time (sec)') ; ylabel('Roll Angle (deg)');
legend('Uncontrolled','Controlled')

figure('Name','Pitch')
set(gcf,'position',[550 220 800 615])
hold on
plot(t,ptp_uncontrolled(:,2))
plot(t,ptp_controlled(:,2),'r-')
xlabel('Time (sec)') ; ylabel('Pitch Angle (deg)');
legend('Uncontrolled','Controlled')

figure('Name','Yaw')
set(gcf,'position',[550 220 800 615])
hold on
plot(t,ptp_uncontrolled(:,3))
plot(t,ptp_controlled(:,3),'r-')
xlabel('Time (sec)') ; ylabel('Yaw Angle (deg)');
legend('Uncontrolled','Controlled')

%%% Compute sensitivities
std_uncontrolled = std(ptp_uncontrolled)
std_controlled   = std(ptp_controlled)

norm_uncontrolled= norm(std_uncontrolled)
norm_controlled  = norm(std_controlled)

