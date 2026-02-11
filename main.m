close all;
clear; 
clc;
pkg load quaternion
#source "./tools/utilities/geometry_helpers_3d.m"
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


#this is the robot wrt world
x_robot_wrt_world=[0.0005 0.0000 0.0000 -0.0000 -0.0000 0.0002 1.0000]';

#we use the function invertPose to obtain the pose of 
#the world wrt the robot
x_true = invertPose(x_robot_wrt_world);
#x_guess = x_true #for testing
x_guess = [0 0 0 0 0 0 1];


#function performing icp for one epoch
function x_result = icpOneEpoch(fid, fid_traj_check,x_guess,P_world,cameras);
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

for i=1:999
    x_result = icpOneEpoch(fid,fid_traj_check,x_guess,P_world,cameras);
    x_guess = x_result';
    disp('---------------------------------------------')
endfor
















#testdistancemap
#project points 
#pose = [0.0005 0.0000 0.0000 -0.0000 -0.0000 0.0002 1.0000]';
#x = invertPose(pose);
#t = x(1:3);
#q = x(4:7);
#R = rotationMatrixFromQuaternion(q(1),q(2),q(3),q(4));
#[P_Proj_c0, P_Proj_c1, P_Proj_c2] = projectLandmarksInCameraDistMap(P_world,cameras,R,t);
#[D_c0,Parent_c0] = createDistanceMapNew(P_Proj_c0);
#disp(Parent_c0(123,477));





#try
#[ cid: 0 lid: 26 imp: 477.0347 123.0206 ]
#pose = [0 0 0 0 0 0 1]
#t = pose(1:3);
#q = pose(4:7);
#R = rotationMatrixFromQuaternion(q(1),q(2),q(3),q(4));
#[epoch,measurements] = loadMeas(fid);
#Z = measurements;
#A = getAssociationMatrix(P_world,Z,cameras,R,t);
#a = associateMeasurements(A,Z)
#likelihoods = computeLikelihood(P_world,ciccio,K,T,R,t)






#close file
fclose(fid);













