function [pose] = read_gt_trajectory(fid)
    pose = [];
    line = fgetl(fid); #take new line

    if line == -1; #if we reached the end of file 
        return;
    end

    while line(1) == '#'; #do not consider comments
        line = fgetl(fid);
    end

    data = strsplit(line); 

    for i=4:10
        el = str2double(data{i});
        
        pose = [pose; el];
    endfor
    pose = pose';
    
end
