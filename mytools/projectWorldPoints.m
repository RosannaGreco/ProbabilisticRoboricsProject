

#input: 
#  -p in world coordinates
#   K, T of the camera 
#   R,t robot rotation matrix and translation vector
#output: p projected in the camera screen
function[p_projected,pcam_hat, p_cameraframe, p_robotframe] = projectWorldPoints(p,K,T,R,t)
    #p = point in the world frame
    #R = matrix obtained from the quaternions
    #point in the robot frame
   
    p_robotframe = R*p+t;
    #point in the camera frame
    R_camera = T(1:3, 1:3); #rotational part
    t_camera = T(1:3, 4) ; #translational part
    p_cameraframe = R_camera'*(p_robotframe - t_camera);
    #apply K and project
    
    pcam_hat = K*p_cameraframe; #apply K matrix
    p_projected = pcam_hat(1:2)/pcam_hat(3);
    
   
    

end

function [p_img] = projection(p)
    p_img = p(1:2)/p(3);
    
   # xcam = p(1);
   # ycam = p(2);
   # zcam = p(3);
   # x = xcam/zcam;
   # y = ycam/zcam;
   # p_img = [x;y];
end