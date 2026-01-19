source "./tools/utilities/geometry_helpers_3d.m"
source "./mytools/quaternions_helper.m"
source "./mytools/cameras_helper.m"
addpath('./mytools');




function [e,J]=errorAndJacobian(x,p,z,cid, cameras)

  t=x(1:3); #translation part
  q=x(4:7); #rotation part (quaternions)
  q=q/norm(q); #normalize quaternion
  #get rotation matrix from quaternion
  R = rotationMatrixFromQuaternion(q(1),q(2),q(3),q(4));
  #cameras 
  id = cid;#take the id of the camera that saw the point
  c = cameras(id+1); #note: cameras(1) is corresponds to camera id 0
  K = c.K;
  T = c.T;
  #translation and rotation of the camera wrt robot
  T_inv = inv(T);
  t_camera_robot = T_inv(1:3,4);
  R_camera_robot = T_inv(1:3, 1:3); 

  
  #prediction and error
  #project the world in camera coordinates using a function defined in the folder 'mytools'
  #we also take p_camera frame, i.e. the point expressed in the camera frame (in 3 coordinates)
  [z_hat,p_cam_hat,p_cameraframe] = projectWorldPoints(p,K,T,R,t);
   
  #ERROR----------------------------------------
  #scaling wrt pixel
  #height and width are shared among cameras
  width = 640;
  height = 480;
  d = diag([1/width, 1/height]);
  #error (normalized in pixel)
  e = d*(z_hat-z);
  #JACOBIAN--------------------
  #note:
  #p_cam_hat = K*p_cameraframe
  #p_cameraframe = X*p_world (with X transformation from camera to world)

  #Jproj computation (using a function defined in cameras_helper)     
  Jproj = getJproj(p_cam_hat); 
  #Jicp computation (using a code similar to the one provided by the professor)
  Jicp = zeros(3,6);
  Jicp(:,1:3)=eye(3);  
  px = skew(p_cameraframe);
  Jicp(:,4:6) = -px; 
  
  #final Jacobian
  J = Jproj*K*Jicp;
  #J = J; #multiplying by Adjoint matrix to go in the robot frame
  
  
endfunction

#quaternion update: this function is used to convert a rotational offset
#expressed wrt 3 angles in quaternions and update the state values qx,qy,qz,qw
#input: q (current value in quaternions), dtheta (rotation expressed wrt 3 angles)
#output: q_update (updated quaternion)
function [q_update] =quaternion_update(q,dtheta);
  if dtheta < 1e-6 #handling small rotations
      dq = [1;0;0;0]; 
  else
    theta = norm(dtheta);
    ax = dtheta/theta; #rotation axis
    dq = [cos(theta/2); ax*sin(theta/2)]; #compute offset in terms of quaternions
  end
  #updating values
  q_update = quaternion_multiplication(q,dq);
  q_update = q_update/norm(q_update);
  
endfunction



function [x]= doIcp(x_guess,P, Z, num_iterations, cameras)
  x=x_guess; #initial guess
  #chi_stats=zeros(1,num_iterations); #ignore this for now
 
  for (iteration=1:num_iterations)
   #init H and b
    H=zeros(6,6);
    b=zeros(6,1);
    chi_stats(iteration)=0; 
    for (i=1:length(Z)) #we iterate on the struct containing the measurements
      
      m = Z(i); #take single measurement
      if m.cid != 2 #testing with a camera
        continue;
      endif
      id = m.lid + 1;#in the matrix P, landmark j is on the (j+1)th column
      z = [m.pos.x;m.pos.y]; #for now, let's discard the camera part blabla
      cid = m.cid;
     
      [e,J] = errorAndJacobian(x, P(:,id), z, cid, cameras); #compute e and J using above function
      #chi_stats(iteration)+=chi;
     

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
