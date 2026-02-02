
source "./tools/utilities/geometry_helpers_3d.m"
source "./mytools/quaternions_helper.m"
source "./mytools/cameras_helper.m"
addpath('./mytools');




function [e,J]=errorAndJacobian(x,p,z,cid, cameras)
  t=x(1:3); #translation part
  q=x(4:7); #rotation part (quaternions)
  q = q / norm(q);
  #q=q/norm(q); #normalize quaternion
  #get rotation matrix from quaternion
  R = rotationMatrixFromQuaternion(q(1),q(2),q(3),q(4));
  #cameras 
  id = cid;#take the id of the camera that saw the point
  c = cameras(id+1); #note: cameras(1) is corresponds to camera id 0
  K = c.K;
  T = c.T;
  

  #prediction and error
  #project the world in camera coordinates using a function defined in the folder 'mytools'
  #we also take p_camera frame, i.e. the point expressed in the camera frame (in 3 coordinates)
  [z_hat,p_cam_hat,p_cameraframe,p_robotframe] = projectWorldPoints(p,K,T,R,t);
   
  #ERROR----------------------------------------
  e = z_hat - z;

  
  
  #JACOBIAN--------------------
  #note:
  #p_cam_hat = K*p_cameraframe
  #p_cameraframe = X*p_world (with X transformation from camera to world)
  
  #corrected: 
  R_camera = T(1:3,1:3);
  Jproj = getJproj(p_cam_hat) ;
  Jicp(:,1:3) = R_camera';
  px = skew(p_robotframe);
  Jicp(:,4:6) = -(R_camera'*px);
  J = Jproj*K*Jicp;

  



  
endfunction

#quaternion update: this function is used to convert a rotational offset
#expressed wrt 3 angles in quaternions and update the state values qx,qy,qz,qw
#input: q (current value in quaternions), dtheta (rotation expressed wrt 3 angles)
#output: q_update (updated quaternion)
function [q_update] =quaternion_update(q,dtheta);
    #convert in rotation matrices 
    rx=Rx(dtheta(1));
    ry=Ry(dtheta(2));
    rz=Rz(dtheta(3));
    Rtot = rz*ry*rx;
    #convert in quaternions
    dq = quaternionFromRotationMatrix(Rtot);  
    #update values
    dq = dq/norm(dq);
    q_update = quaternion_multiplication(q,dq);
    q_update = q_update / norm(q_update);
  
endfunction



function [x]= doIcp(x_guess,P, Z, num_iterations, cameras)
  x=x_guess; #initial guess
  #chi_stats=zeros(1,num_iterations); #ignore this for now
  
  for (iteration=1:num_iterations)
   
   #init H and b
    H=zeros(6,6);
    b=zeros(6,1);
    #chi_stats(iteration)=0; 
    for (i=1:length(Z)) #we iterate on the struct containing the measurements
      
      m = Z(i); #take single measurement
      
      id = m.lid + 1;#in the matrix P, landmark j is on the (j+1)th column
      z = [m.pos.x;m.pos.y]; 
      cid = m.cid;
      
      [e,J] = errorAndJacobian(x, P(:,id), z, cid, cameras); #compute e and J using above function
      
     
      e = e(:);
      H+=J'*J;  
      b+=J'*e;
      
    endfor
    
    H+=eye(6);
    dx=-H\b;
    
    #update translational part
    x(1:3) += dx(1:3); 
    #update rotational part using the function above
    x(4:7) = quaternion_update(x(4:7),dx(4:6)); 
    
    
  endfor
endfunction



