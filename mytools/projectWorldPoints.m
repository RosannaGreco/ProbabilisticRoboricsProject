

#input: 
#  -p in world coordinates
#   K, T of the camera 
#   R,t robot rotation matrix and translation vector
#output: p projected in the camera screen
function[p_projected,p_cameraframe] = projectWorldPoints(p,K,T,R,t)
    #point in the robot frame
    p_robotframe = R*p+t;
    #point in the camera frame
    p_robotframe_h = [p_robotframe; 1];       # convert in homogeneous coordinates
    p_cameraframe_h = inv(T) * p_robotframe_h ;      # apply T
    p_cameraframe = p_cameraframe_h(1:3);
    #apply K and project
    pcam_hat = K*p_cameraframe; #apply K matrix
    p_projected = projection(pcam_hat);
end

function [p_img] = projection(p)
    xcam = p(1);
    ycam = p(2);
    zcam = p(3);
    x = xcam/zcam;
    y = ycam/zcam;
    p_img = [x;y];
end