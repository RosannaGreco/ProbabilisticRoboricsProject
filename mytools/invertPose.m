#this function takes a pose 
#i.e. robot wrt world
#and expresses it as world wrt robot
function x_inv = invertPose(x)
    t=x(1:3);      
    q=x(4:7);      
    q=q/norm(q); 
    #inverse rotation
    q_inv = [-q(1) -q(2) -q(3) q(4)];
    R_inv = rotationMatrixFromQuaternion(q_inv(1),q_inv(2),q_inv(3),q_inv(4));
    #inverse translation
    t_inverse = -R_inv*t;
    #x_inv
    x_inv(1:3) = t_inverse;
    x_inv(4:7) = q_inv;
end

