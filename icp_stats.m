#file used to print the error on a single iteration

close all;
clear; 
clc;
source "./mytools/quaternions_helper.m"
source "./mytools/map_builder.m"
source "./mytools/measurements_handler.m"
source "./icp.m"
source "./mytools/dataAssociation.m"
addpath('./mytools');

#load data from the map and define Pworld
landmarks = loadMap('map.dat');
disp('map loaded');
P_world = build_P_world_matrix_from_map(landmarks);

#define camera parameters
[c0,c1,c2] = defineParams();
cameras = [c0,c1,c2]; #array of structs 
disp('camera parameters loaded');

#compute the measurements
fid = fopen('meas.dat', 'r');
fid_traj_check = fopen('traj.dat', 'r');
[epoch,measurements] = loadMeas(fid);
Z = measurements;

iterations=100;
gt_pose = read_gt_trajectory(fid_traj_check);



chi_stats = zeros(1, iterations);
num_inliers = zeros(1, iterations);


# test with a good initial guess
x_guess=[0,0,0,0,0,0,1]';


X_guess=v2t_quaternion(x_guess);
[X_result, chi_stats,  num_inliers] = doIcp(X_guess, P_world, Z, iterations, cameras);;


figure;
plot(log(chi_stats), '-o', 'LineWidth', 2);
xlabel('iteration');
ylabel('error');
title('error evolution (good initial guess)');
grid on;


hold off
printf("press [Enter] to exit\n");
pause();

fclose(fid);
fclose(fid_traj_check);


