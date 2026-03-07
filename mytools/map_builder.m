#this function is used to load the landmarks from the file
#input : a file path
#output: 
#   for each landmark, a struct containing:
#   id,x,y,z
function [landmarks] = loadMap(filepath)
    fid = fopen(filepath, 'r');
    fgetl(fid);#ignore first line
    landmarks = [];
    temp = {};
    line = fgetl(fid);
    while ischar(line)
        
    #split line
        data = strsplit(line);

        landmark = struct();
    
        landmark.id = str2double(data{2});
        landmark.x = str2double(data{4});
        landmark.y = str2double(data{5});
        landmark.z = str2double(data{6});

        temp{end+1} = landmark;
        line = fgetl(fid);
    end
    fclose(fid);
    landmarks = [temp{:}];
end

#this function builds a matrix where each column represents a point
#reminder: in octave arrays start with 1, while the indeces start with 0
function P = build_P_world_matrix_from_map(landmarks);
    n_points = length(landmarks);
    P = zeros(3, n_points);
    for p=1:n_points;
        point = landmarks(p);
        x = point.x;
        y = point.y;
        z = point.z;
        id = point.id + 1; #we start the matrix with column 1
        #remember that all the ids will be shifted
        P(:, id) = [x; y; z];
    endfor
end
