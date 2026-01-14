#this function takes in input quaterions and returns a rotation matrix
#input: qx qy qz qw
#output: rotation matrix R
function [R] = rotationMatrixFromQuaternion(q_x,q_y,q_z,q_w);
    R = zeros(3,3); 
    q = [q_x q_y q_z q_w];
    q = q/norm(q);

    q_x = q(1);
    q_y = q(2);
    q_z = q(3);
    q_w = q(4);

    r11= 2*(q_w^2 + q_x^2) -1;
    r12 = 2*(q_x*q_y - q_w*q_z);
    r13 = 2*(q_x*q_z + q_w*q_y);
    r21 = 2*(q_x*q_y + q_w*q_z);
    r22= 2*(q_w^2 + q_y^2) -1;
    r23 = 2*(q_y*q_z - q_w*q_x);
    r31 = 2*(q_x*q_z - q_w*q_y);
    r32 = 2*(q_y*q_z + q_w*q_x);
    r33= 2*(q_w^2 + q_z^2) -1;
    R = [r11 r12 r13; r21 r22 r23; r31 r32 r33];
end 

#from 7d vector (translation | quaternions) to homogeneous matrix
#this function is a modified version of v2t (found in the code provided during the course)
#input: 7d vector v
#output: matrix T
function T=v2t_quaternion(v)
    q = v(4:7);
    q = q/norm(q); #normalize quaternion
    R = rotationMatrixFromQuaternion(q(1),q(2),q(3),q(4));
    T=eye(4);
    T(1:3,1:3)=R;
    T(1:3,4)=v(1:3);
endfunction;

#this function is used to multiply two quaternions
#input: q1 q2
#output: q_new
function q_new = quaternion_multiplication(q1,q2)
    q2_w = q2(4); 
    q2_xyz = q2(1:3);
    q1_w = q1(4); 
    q1_xyz = q1(1:3);
  
    q_new = zeros(4,1); #init updated quaternion
    q_new(4) = q2_w*q1_w - dot(q2_xyz, q1_xyz); #update qw part
    q_new(1:3) = q2_w*q1_xyz + q1_w*q2_xyz + cross(q2_xyz,q1_xyz); #update q_xyz part
    q_new = q_new/norm(q_new); #normalize
endfunction

function x_inv = invert_pose(x)
    t = x(1:3);
    q = x(4:7);
endfunction
