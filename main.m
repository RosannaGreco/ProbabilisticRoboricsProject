close all;
clear; 
clc;
#source "./tools/utilities/geometry_helpers_3d.m"
source "./mytools/quaternions_helper.m"
source "./mytools/map_builder.m"
source "./mytools/measurements_handler.m"
source "./icp.m"
addpath('./mytools');

#load data from the map and define Pworld
landmarks = loadMap('map.dat');
disp('map loaded');
P_world = build_P_world_matrix_from_map(landmarks);

#define camera parameters
[c0,c1,c2] = defineParams();
cameras = [c0,c1,c2]; #array of structs 
disp('camera parameters loaded');


#initial guess for robot pose
#ideal position of world w.r.t robot
x_true=[0.0005 0.0000 0.0000 -0.0000 -0.0000 0.0002 1.0000]';
#let's add a small offset
x_offset = [0.2 0.2 0.2]'; #small translation
q_offset = [0.0 0.0 0.008 0.9]'; #small rotation
q_offset = q_offset/norm(q_offset);
x_guess(1:3) = x_true(1:3) + x_offset;
x_guess(4:7) = quaternion_multiplication(x_true(4:7), q_offset);



#load epoch data--------------------------------------------------
#read file to get measurements
fid = fopen('meas.dat', 'r');

#for each epoch


#load measurements
[epoch,measurements] = loadMeas(fid);
disp('new data loaded from cameras! :)')
epoch = epoch
n_seen_points= length(measurements) #number of measurements for this epoch

Z = measurements; 
iterations=100;
[x_result] = doIcp(x_guess', P_world, Z, iterations, cameras);
disp('result:')
fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', x_result');
    





#close file
fclose(fid);













