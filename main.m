close all;
clear; 
clc;
addpath('./mytools');

%load data 
landmarks = loadMap('map.dat');
disp('map loaded');
%define camera parameters
cam_parameters = defineParams();
disp('camera parameters loaded');

%load epoch data--------------------------------------------------
%read file to get measurements
fid = fopen('meas.dat', 'r');
[epoch,measurements] = loadMeas(fid);


n_points= length(measurements); #number of measurements for this epoch

#the world points are the landmarks
P_world = landmarks;


#initial guess
x_guess=[0,0,0,0,0,0,0]'; #using quaternions




#close file
fclose(fid);





