
addpath('./mytools');


#project all the points in each camera (considering the pose)
#input:
# P 
# cameras struct 
# R rotation matrix of the pose (world wrt robot)
# t translational part of the pose (world wrt robot)
#output: 
# P_Proj_c0, P_Proj_c1, P_Proj_c2
function [P_Proj_c0, P_Proj_c1, P_Proj_c2] = projectLandmarksInCamera(P,cameras,R,t)
    [dimpoint,N] = size(P);
  
    #camera 0
    c = cameras(1); #note: cameras(1) is corresponds to camera id 0
    K = c.K;
    T = c.T;
    P_Proj_c0 = projectWorldPointsM(P,R,t,K,T);
   

    #camera 1
    c = cameras(2); #note: cameras(1) is corresponds to camera id 0
    K = c.K;
    T = c.T;
    P_Proj_c1 = projectWorldPointsM(P,R,t,K,T);


    #camera 2
    c = cameras(3); #note: cameras(1) is corresponds to camera id 0
    K = c.K;
    T = c.T;
    P_Proj_c2 = projectWorldPointsM(P,R,t,K,T);

endfunction




#this function builds the association matrix 
#inputs: 
# matrices containing vectors projected in the 3 cameras
# Z 
# R,t
#output: A 
function A = getAssociationMatrix(P_Proj_c0, P_Proj_c1,P_Proj_c2,Z,R,t)
    [dimpoint,N] = size(P_Proj_c0); #number of points
    M= size(Z); #number of measurements
    #init A 
    A = ones(M,N)*1e3;

    for (m=1:length(Z)) #for each measurement
        meas = Z(m); 
        z = [meas.pos.x;meas.pos.y];  #retrieve coordinates
        cid = meas.cid; #retrieve camera index
        
        #pick right set of projected points
        if cid == 0
            P_Proj = P_Proj_c0;
        elseif cid == 1 
            P_Proj = P_Proj_c1;
        elseif cid == 2
            P_Proj = P_Proj_c2;
        endif
        
        #computing cost
        E = P_Proj - z;              
        A(m,:) = sqrt(sum(E.^2,1)); #norm of the error in pixel

    endfor

end






#this function creates a vector of associations in the format 
#[measurement id, proposed landmark id, association matrix value]
#input: A, Z
#output: associations 
function associations = associateMeasurements(A,Z,tau)
    gating_tau = tau; 
    
    [M,N] = size(A);
    associations = zeros(M,3);
    #gating
    for m = 1:M 
        #lid = Z(m).lid;
        [a_mn,min_index] = min(A(m,:)); 
        
        if(a_mn < gating_tau)
            associations(m,:) = [m,min_index,a_mn];
        else
            associations(m,:) = [m,0,a_mn];
        endif
    endfor
   
     
end

#this is a modified version of projectWorldPoints which does 
#the same thing, but takes in input a matrix of points. 
#It is used to make the computation more efficient
function PProjected = projectWorldPointsM(P,R,t,K,T)
    P_robot = R*P + t;
    R_camera = T(1:3,1:3);
    P_camera = R_camera'*(P_robot - T(1:3,4));



    P_cam_hat = K*P_camera;  
    PProjected = P_cam_hat(1:2,:) ./ P_cam_hat(3,:);
endfunction


