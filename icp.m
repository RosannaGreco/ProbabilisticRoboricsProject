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
  R_camera = T(1:3, 1:3);
 
  #prediction and error

  #project the world in camera coordinates using a function defined in the folder 'mytools'
  #we also take p_camera frame, i.e. the point expressed in the camera frame (in 3 coordinates)
  [z_hat,p_cameraframe,p_robotframe] = projectWorldPoints(p,K,T,R,t);
     #error 

  #scaling wrt pixel
  #height and width are shared among cameras
  width = 640;
  height = 480;
  e = [(z_hat(1)-z(1))/width; (z_hat(2)-z(2))/height];
  #JACOBIAN
  #Jproj computation (using a function defined in cameras_helper)     
  Jproj = getJproj(p_cameraframe);
  #Jicp computation (using a code similar to the one provided by the professor)
  Jicp = zeros(3,6);
  Jicp(:,1:3)=eye(3); 
  px = skew(p_cameraframe);
  Jicp(:,4:6) = -px;
  
  #final Jacobian
  J = Jproj*K*Jicp;

  
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
      id = m.lid + 1;#in the matrix P, landmark j is on the (j+1)th column
      z = [m.pos.x;m.pos.y]; #for now, let's discard the camera part blabla
      cid = m.cid;
      [e,J] = errorAndJacobian(x, P(:,id), z, cid, cameras); #compute e and J using above function
      #chi_stats(iteration)+=chi;
     
      H+=J'*J;
      b+=J'*e;
    endfor
    H+=eye(6);#*12; 
    dx=-H\b;
    #update translational part
    x(1:3) += dx(1:3); 
    #update rotational part using the function above
    x(4:7) = quaternion_update(x(4:7),dx(4:6)); 
    
  endfor
endfunction
