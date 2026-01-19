#this function takes a point in camera coordinates and applies the projection
#operation to obtain pixel coordinates
function [p_img] = projection(p)
    xcam = p(1);
    ycam = p(2);
    zcam = p(3);
    x = xcam/zcam;
    y = ycam/zcam;
    p_img = [x;y];
end
#this function is used to compute Jproj
#input: point p
#output: matrix Jproj
function [Jproj] = getJproj(p)
    x = p(1);
    y = p(2);
    z = p(3);
    Jproj = [1/z 0 -x/(z^2);
             0 1/z -y/(z^2)];
end

