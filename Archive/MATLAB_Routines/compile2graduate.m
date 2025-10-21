%%%%run TERN Simulation%%%%%%
%purge
clear
close all
clc

%tic
addpath 'plotting/'
parameters

%%%COMPILE CODE
if COMPILECODE
  system('make rebuild');
end

%%%RUN CODE
tic
if RUNCODE
  system('rm Output_Files/*.OUT');
  system('rm Output_Files/*.txt');
  system('./LinuxRun.exe Input_Files/TOMAD.ifiles');
end
toc
%toc
addpath 'plotting/'
plotTOMAD
% save_plots;

%pause
%plotActuators
% Visualizer
%ParafoilViz
