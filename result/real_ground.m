% display ground truth of KITTI odometry dataset sequence 00
% include sequence and read from poses
filename = 'C:\Users\mxd-2\OneDrive\Desktop\coursework\poses\00.txt';
fid = fopen(filename);
fseek(fid, 0, 'bof');
lastposition = ftell(fid);
disp(['start position:' , num2str(lastposition)]);

groundtruth = [];
while fgetl(fid) ~= -1      % end of line check
    fseek(fid, lastposition, 'bof');
    line = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f \n', 1);
    line = [line{:}];
    transform = vec2mat(line, 4);
    
    groundtruth = [groundtruth; [transform(1,4), transform(3,4)]];
    lastposition = ftell(fid);
    disp(['lastposition:', num2str(lastposition)]);
end

% display ground truth
scatter(groundtruth(:,1), groundtruth(:,2),1);

fclose(fid);
