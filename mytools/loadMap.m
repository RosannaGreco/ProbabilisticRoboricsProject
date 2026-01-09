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



