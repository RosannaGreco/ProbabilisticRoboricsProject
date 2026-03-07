
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

