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



#function performing icp for one epoch
#inputs: 
# fids of the files to read measurements and write computed poses (see below)
# x_guess (7d vector format)
# P_world (matrix with landmarks in the world frame)
# cameras (struct with camera parameters)
#outputs: 
# x_result: estimated pose (world wrt robot)
# x_robotpose: estimated pose (robot wrt world)
# e: trajectory error
function [x_result,x_robotpose, e] = icpOneEpoch(fid, fid_traj_check,x_guess,P_world,cameras);
    [epoch,measurements] = loadMeas(fid); #load measurements
    epoch = epoch
    n_seen_points= length(measurements) 
    Z = measurements;
    iterations=100;
    gt_pose = read_gt_trajectory(fid_traj_check);
    
    X_guess = v2t_quaternion(x_guess'); #convert in transformation matrix
    #do icp
    [X_result] = doIcp(X_guess, P_world, Z, iterations, cameras);

    x_result = t2v_quaternion(X_result); #convert in 7d vector
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




#load epoch data
#read file to get measurements
fid = fopen('meas.dat', 'r');
#other files: 
fid_traj_check = fopen('traj.dat', 'r'); #to read the gt of the trajectory (useful for traj error computation)
fid_write_poses = fopen('poses.dat','w'); #to write the obtained poses
fid_write_err = fopen('error.dat','w'); #to write the obtained trajectory error

#RANSAC PART: Global pose estimation
[epoch,measurements] = loadMeas(fid); #load measurements of the first epoch

#camera 0
cam0 = cameras(1);
K0 = cam0.K;
T0 = cam0.T;

[t,q] = Ransac(measurements,P_world,K0,T0, cameras); 

x_guess_inv = [t(:)' q(:)']; #pose of the world wrt robot
x_guess_r = invertPose(x_guess_inv'); #pose of the robot wrt world
disp('ransac guess:')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', x_guess_r); 
#compute trajectory error
gt_pose = read_gt_trajectory(fid_traj_check) #retrieve gt pose
err = gt_pose - x_guess_r; #compute error

disp('error:')
    fprintf('%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', err); 

#print first pose in file storing poses 
fprintf(fid_write_poses, 'epoch: %d pose: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', 0,x_guess_r(1),...
    x_guess_r(2), x_guess_r(3), x_guess_r(4), x_guess_r(5), x_guess_r(6), x_guess_r(7));

#print error in the file storing traj. errors
fprintf(fid_write_err, 'epoch: %d pose: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', 0,err(1),...
    err(2), err(3), err(4), err(5), err(6), err(7));

x_guess = x_guess_inv;


#ICP 
for i=2:1000
    [x_result,x_robotpose,err] = icpOneEpoch(fid,fid_traj_check,x_guess,P_world,cameras);
    x_guess = x_result';
    disp('---------------------------------------------')
    #print on file
    #pose in poses.dat
    fprintf(fid_write_poses, 'epoch: %d pose: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', i-1,x_robotpose(1),...
    x_robotpose(2), x_robotpose(3), x_robotpose(4), x_robotpose(5), x_robotpose(6), x_robotpose(7));
    #error in error.dat
    fprintf(fid_write_err, 'epoch: %d pose: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', i-1,err(1),...
    err(2), err(3), err(4), err(5), err(6), err(7));
endfor


#close file
fclose(fid);
fclose(fid_traj_check);
fclose(fid_write_err);
fclose(fid_write_poses);












