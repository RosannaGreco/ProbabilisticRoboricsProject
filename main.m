close all;
clear; 
clc;
pkg load quaternion
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


#this is the robot wrt world
x_robot_wrt_world=[0.0005 0.0000 0.0000 -0.0000 -0.0000 0.0002 1.0000]';

#we use the function invertPose to obtain the pose of 
#the world wrt the robot
x_true = invertPose(x_robot_wrt_world);
#x_guess = x_true #for testing
x_guess = [0 0 0 1 0 0 0];
#x_guess =  [-0.0066   0.003        0.08        0        0  -0.0005   1.0000];



#function performing icp for one epoch
function x_result = icpOneEpoch(fid,x_guess,P_world,cameras);
    [epoch,measurements] = loadMeas(fid); #load measurements
    epoch = epoch
    n_seen_points= length(measurements) 
    Z = measurements;
    iterations=100;
    #do icp
    [x_result] = doIcp(x_guess', P_world, Z, iterations, cameras);
    #display result
    disp('result (world wrt robot):')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', x_result'); 
    x_robotpose = invertPose(x_result);
    disp('result (robot wrt world):')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', x_robotpose'); 

end

#this function is used to perform a test on one of the points
#with projectWorldPoints
function p_c = testpoint(cameras,P_world)
    x = [-0.0005 -0.0 -0.0 0.0 0.0 -0.0002 1.0]; #first pose, but
    #espressed as world wrt robot
    c = cameras(2);
    p = P_world(:,27);
    K = c.K;
    T = c.T;
    q = x(4:7);
    R = rotationMatrixFromQuaternion(q(1),q(2),q(3),q(4));
    t = x(1:3);
    t = t(:);
    disp(' correct point : 471.7487 126.9584')
    p_c = projectWorldPoints(p,K,T,R,t)
end






#load epoch data--------------------------------------------------
#read file to get measurements
fid = fopen('meas.dat', 'r');

for i=1:1
    x_result = icpOneEpoch(fid,x_guess,P_world,cameras);
    x_guess = x_result';
    disp('---------------------------------------------')
endfor

#pc = testpoint(cameras,P_world);





#close file
fclose(fid);













