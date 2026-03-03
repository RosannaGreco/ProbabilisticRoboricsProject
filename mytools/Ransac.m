#RANSAC
function[t,q] = Ransac(Z,P,K,T, cameras)
    best_num_inliers = 0;
    #init with identity
    R_best = eye(3);
    t_best = [0;0;0];
    X_best = eye(4);
    best_cost = 800;
    inlier_threshold = 10;
    #retrieve camera 0 measurements (as a struct and as a matrix)
    Z_camera0 = retrieveCamera0Measurements(Z,0);
    Z_camera0_struct = retrieveCamera0MeasurementsStruct(Z,0);
    n_measurements = length(Z_camera0_struct);
    n_landmarks = size(P,2);
    for i = 1:2000
        #build candidate correspondences
        #1. project all landmarks in camera 0
        projected_P = projectWorldPointsVector(P,R_best,t_best,K,T);
        #2. use association matrix to get some candidate correspondences
        A = getAssociationMatrix(projected_P,[],[], Z_camera0_struct,R_best,t_best);
        associations = associateMeasurements(A, Z_camera0_struct); 
        candidate_correspondences= getCandidateCorrespondences(associations);
        n_correspondences = size(candidate_correspondences,1);

        if n_correspondences < 15
            continue;
        endif
        #extract random correspondences from the candidates
        random_idxs = randperm(n_correspondences,15);
        m_idxs = candidate_correspondences(random_idxs,1);
        l_idxs = candidate_correspondences(random_idxs,2);
        sampled_measurements = Z_camera0_struct(m_idxs);
        sampled_landmarks = P(:, l_idxs);
        #perform icp (using the code used for point 1 and 2)
        x_guess = zeros(1,7);
        x_guess(1:3) = t_best;
        x_guess(4:7) = quaternionFromRotationMatrix(R_best);
        new_x = doIcpRANSAC(x_guess', sampled_landmarks, sampled_measurements, 10, cameras);
        #project points in the new pose
        q_new = new_x(4:7);
        R_new = rotationMatrixFromQuaternion(q_new(1),q_new(2),q_new(3),q_new(4));
        t_new = new_x(1:3);
        P_newcameraframe = R_new*P+ t_new;
        new_projected_P = projectWorldPointsVector(P,R_new,t_new,K,T);
        #compute inliers
        A_new = getAssociationMatrix(new_projected_P,[],[], Z_camera0_struct,R_new,t_new);
        associations_new = associateMeasurements(A_new, Z_camera0_struct); 
        inliers = 0;
        cost = 0;
        inliers_idxs = [];
        for a=1:size(associations_new,1) #for each association
            assoc = associations_new(a,:);
            if(assoc(3)< inlier_threshold)
                inliers+=1;
                inliers_idxs = [inliers_idxs, a];
                cost+= assoc(3);
            endif
        endfor
        #update if we found a better result
        if (inliers > best_num_inliers) && (cost < best_cost)
            R_best = R_new;
            t_best = t_new;
            best_cost = cost;
            best_num_inliers = inliers;
            
        endif

    endfor
    R_robot = R_best;
    t_robot = t_best;
    best_num_inliers
    t = t_robot;
    q = quaternionFromRotationMatrix(R_robot);
    
     

endfunction




#takes candidate associations using the association matrix
function candidate_correspondences= getCandidateCorrespondences(associations)
    n = size(associations,1);
    candidate_correspondences = [];
    for i=1:n 
        a = associations(i, :);
        if (a(2) != 0)
            measurement = a(1);
            point_lid = a(2);
            candidate_correspondences = [candidate_correspondences; measurement, point_lid];
        endif
    endfor 
endfunction








#this function extracts all the points seen by camera 0 
#input: Z, c (cid of desired camera)
#output: points seen by the camera(2d coordinates)
function z_camera0 = retrieveCamera0Measurements(Z,c)
    z_camera0 = [];
    for (i=1:length(Z)) 
        m = Z(i);
        cid = m.cid;
        if (cid == c)
            x = m.pos.x;
            y = m.pos.y;
            z_camera0 = [z_camera0;x,y];
            
        endif
    endfor
    z_camera0 = z_camera0';
endfunction



#this function extracts all the points seen by camera 0, but in a struct
#input: Z, c (cid of desired camera)
#output: struct containing points seen by the camera(2d coordinates)
function z_camera0_struct = retrieveCamera0MeasurementsStruct(Z,c)
    z_camera0_struct = [];
    for (i=1:length(Z)) 
        m = Z(i);
        cid = m.cid;
        if (cid == c)
            x = m.pos.x;
            y = m.pos.y;
            z_camera0_struct = [z_camera0_struct;m];
            
        endif
    endfor
endfunction
#-------------------------------------------------------------------------------
#this is a modified version of doIcp not handling the data association part 
#(in this case, it is handled in the ransac loop)
function [x]= doIcpRANSAC(x_guess,P, Z, num_iterations, cameras)
  x=x_guess; #initial guess
  chi_stats=zeros(1,num_iterations); 
  num_inliers=zeros(1,num_iterations); 
  kernel_threshold = 5;
  for (iteration=1:num_iterations)
   
   #init H and b
    H=zeros(6,6);
    b=zeros(6,1);
    chi_stats(iteration)=0; 
   
    #data association part
    t = x(1:3);
    q = x(4:7);
    R = rotationMatrixFromQuaternion(q(1),q(2),q(3),q(4));
    


    for (i=1:length(Z)) #we iterate on the struct containing the measurements
      
      m = Z(i); #take single measurement

      id = i; #we don't need data association here
      
      z = [m.pos.x;m.pos.y]; 
      cid = m.cid;
      
      [e,J] = errorAndJacobian(x, P(:,id), z, cid, cameras); #compute e and J using above function
      
      
      #kernel treshold part-------------
      kernel_threshold = 1e9; 

      chi=e'*e;
      if (chi>kernel_threshold)
	      e*=sqrt(kernel_threshold/chi);
	      chi=kernel_threshold;
      else
	      num_inliers(iteration)++;
      endif;
      #-----------------------------------

      chi_stats(iteration)+=chi;
     
      
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