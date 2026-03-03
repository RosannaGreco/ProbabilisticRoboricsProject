close all;
clear; 
clc;
source "./mytools/quaternions_helper.m"
source "./mytools/map_builder.m"
source "./mytools/measurements_handler.m"
source "./icp.m"
source "./mytools/dataAssociation.m"
addpath('./mytools');
source "./mytools/epipolar.m"

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
x_guess = [0 0 0 0 0 0 1];


#function performing icp for one epoch
function [x_result,x_robotpose] = icpOneEpoch(fid, fid_traj_check,x_guess,P_world,cameras);
    [epoch,measurements] = loadMeas(fid); #load measurements
    epoch = epoch
    n_seen_points= length(measurements) 
    Z = measurements;
    iterations=100;
    gt_pose = read_gt_trajectory(fid_traj_check);
    
    
    #do icp
    [x_result] = doIcp(x_guess', P_world, Z, iterations, cameras);

    #display result
    #uncomment to see result expressed as world wrt robot
    #disp('result (world wrt robot):')
    #fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', x_result'); 

    #result robot wrt world
    x_robotpose = invertPose(x_result);
    disp('ground truth:')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', gt_pose'); 
    disp('result:')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', x_robotpose'); 

    #error between robot pose and gt 
    e = gt_pose - x_robotpose;

    #handle quaternion representation
    q_gt = gt_pose(4:7);
    q_robotpose = x_robotpose(4:7);
    if dot(q_gt, q_robotpose) < 0
        q_robotpose_changed = -q_robotpose;
        e(4:7) = q_gt-q_robotpose_changed;
    end
    disp('trajectory error:')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', e); 

end




#load epoch data--------------------------------------------------
#read file to get measurements
fid = fopen('meas.dat', 'r');
fid_traj_check = fopen('traj.dat', 'r');
fid_write_poses = fopen('poses.dat','w'); #to write the obtained poses

#for i=1:1000
#    [x_result,x_robotpose] = icpOneEpoch(fid,fid_traj_check,x_guess,P_world,cameras);
#    x_guess = x_result';
 #   disp('---------------------------------------------')
    #print on file
#    fprintf(fid_write_poses, 'epoch: %d pose: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', i-1,x_robotpose(1),...
#    x_robotpose(2), x_robotpose(3), x_robotpose(4), x_robotpose(5), x_robotpose(6), x_robotpose(7));
#endfor
[epoch,measurements] = loadMeas(fid);
cam0 = cameras(1);
cam1 = cameras(2);
K0 = cam0.K;
K1 = cam1.K;
T0 = cam0.T;
T1 = cam1.T;
[t,q] = Ransac(measurements,P_world,K0,T0);
x_guess_inv = [t(:)' q(:)'];
x_guess = invertPose(x_guess_inv');
disp('ransac guess:')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', x_guess_inv);
gt_pose = read_gt_trajectory(fid_traj_check)
err = gt_pose - x_guess_inv;
disp('error:')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', err); 

#for i=1:999
    #[x_result,x_robotpose] = icpOneEpoch(fid,fid_traj_check,x_guess,P_world,cameras);
    #x_guess = x_result';
    #disp('---------------------------------------------')
    #print on file
    #fprintf(fid_write_poses, 'epoch: %d pose: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', i-1,x_robotpose(1),...
    #x_robotpose(2), x_robotpose(3), x_robotpose(4), x_robotpose(5), x_robotpose(6), x_robotpose(7));
#endfor


#close file
fclose(fid);













