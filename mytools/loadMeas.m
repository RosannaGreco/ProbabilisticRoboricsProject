#this function is used to load the measurements at each epoch
#input: file id of an open file
#output: for each measurement, we have a struct containing:
#        epoch, cid, lid, pos (containing x and y on the image)






function [epoch,measurements] = loadMeas(fid)
    measurements = [];
    line = fgetl(fid); #take new line

    if line == -1; #if we reached the end of file 
        return;
    end

    while line(1) == '#'; #do not consider comments
        line = fgetl(fid);
    end

    

    #get epoch 
    epoch = sscanf(line,'EPOCH: %d');
    #get measurements
    line = fgetl(fid); #new line 
    set_of_measurements = regexp(line, '\[(.*?)\]','tokens'); 
    set_of_measurements = [set_of_measurements{:}];
    for b = 1:length(set_of_measurements);
                meas_block = set_of_measurements{b}; #get block
                toks = strsplit(meas_block); #get tokens
                #create our struct
                m = struct();
                m.cid = str2double(toks{3});
                m.lid = str2double(toks{5});
                m.pos.x = str2double(toks{7});
                m.pos.y = str2double(toks{8});
               
                measurements = [measurements; m];
    endfor




end

