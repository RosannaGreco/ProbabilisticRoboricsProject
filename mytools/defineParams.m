%this function defines three structs for the cameras, containing all the parameters
%the data were copied from param.dat
function[camera_parameters] = defineParams()
    camera_parameters = [];
%-----------------------------------------
    c0 = struct();
    c0.K = [120 0 320;
            0 120 240;
            0   0   1];
    c0.min_z = 0.1;
    c0.max_z = 30;
    c0.T = [0   -1    0    0;
            0    0   -1 -0.5;
            1    0    0    0;
            0    0    0    1];%transformation matrix
    camera_parameters = [camera_parameters; c0];
 %---------------------------------------
    c1= struct();
    c1.K = [ 120   0 320; 
            0 120 240;
            0   0   1];
    c1.min_z = 0.1;
    c1.max_z = 30;
    c1.T = [0  -1   0   0;
           0   0  -1 0.5;
           1   0   0   0;
           0   0   0   1];
    camera_parameters = [camera_parameters; c1];
%---------------------------------------------------
    c2 = struct();
    c2.K = [120   0 320;
            0 120 240;
            0   0   1
    ];
    c2.min_z = 0.1;
    c2.max_z = 30;
    c2.T = [0  -1   0   0;
            0   0  -1   0;
            1   0   0 0.5;
            0   0   0   1];
    camera_parameters = [camera_parameters; c2];
end