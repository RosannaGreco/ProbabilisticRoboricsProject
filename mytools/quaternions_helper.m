#this function takes in input quaterions and returns a rotation matrix
#input: qx qy qz qw
#output: rotation matrix R
function [R] = rotationMatrixFromQuaternion(q_x,q_y,q_z,q_w);
    q = [q_x q_y q_z q_w];
    q = q / norm(q); #normalization

    q_x = q(1);
    q_y = q(2);
    q_z = q(3);
    q_w = q(4);

    r11 = 2*(q_w^2  + q_x^2) - 1;
    r12 = 2*((q_x*q_y) -  (q_w*q_z));
    r13 = 2*((q_x* q_z) + (q_w*q_y));

    r21 = 2*((q_x* q_y) + (q_w*q_z));
    r22 = 2*(q_w^2 + q_y^2)-1;
    r23 = 2*((q_y*q_z) - (q_w*q_x));

    r31 = 2*((q_x* q_z) - (q_w*q_y));
    r32 = 2*((q_y* q_z) + (q_w*q_x));
    r33 = 2*(q_w^2 + q_z^2) - 1;

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
    q_new(4) = q1_w*q2_w - dot(q1_xyz, q2_xyz);
    q_new(1:3) = q1_w*q2_xyz + q2_w*q1_xyz + cross(q1_xyz, q2_xyz); #update q_xyz part
    
endfunction

#obtain a rotation matrix from a quaternion
#function q = quaternionFromRotationMatrix(R)
#    qw = sqrt(max(0, 1 + R(1,1) + R(2,2) + R(3,3))) / 2;
#    qx = sgn(R(3,2) - R(2,3)) * sqrt(max(0, 1 + R(1,1) - R(2,2) - R(3,3))) / 2;
#    qy = sgn(R(1,3) - R(3,1)) * sqrt(max(0, 1 - R(1,1) + R(2,2) - R(3,3))) / 2;
#    q_z = sgn(R(2,1) - R(1,2)) * sqrt(max(0, 1 - R(1,1) - R(2,2) + R(3,3))) / 2;
#    q = [qx;qy;q_z;qw];
#    q = q/norm(q);

#end

function s = sgn(x)
    if x < 0
        s = -1;
    else
        s = 1;
    end

end



function q = quaternionFromRotationMatrix(R)
    tr = trace(R);
    if tr > 0
        S = sqrt(tr + 1.0) * 2;
        qw = 0.25 * S;
        qx = (R(3,2) - R(2,3)) / S;
        qy = (R(1,3) - R(3,1)) / S;
        qz = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
        qw = (R(3,2) - R(2,3)) / S;
        qx = 0.25 * S;
        qy = (R(1,2) + R(2,1)) / S;
        qz = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
        qw = (R(1,3) - R(3,1)) / S;
        qx = (R(1,2) + R(2,1)) / S;
        qy = 0.25 * S;
        qz = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
        qw = (R(2,1) - R(1,2)) / S;
        qx = (R(1,3) + R(3,1)) / S;
        qy = (R(2,3) + R(3,2)) / S;
        qz = 0.25 * S;
    end
    q = [qx; qy; qz; qw];
    q = q / norm(q);
end
