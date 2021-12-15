filename = 'C:\Users\mxd-2\OneDrive\Desktop\coursework\pos.txt';
fid = fopen(filename);
fseek(fid, 0, 'bof');
lastposition = ftell(fid);
disp(['start position:' , num2str(lastposition)]);

result = [];
while fgetl(fid) ~= -1
    fseek(fid, lastposition, 'bof');
    line = textscan(fid, '%f %f %f \n', 1);
    line = [line{:}];
    lastposition = ftell(fid);
    result = [result;line];
    disp(['lastposition:', num2str(lastposition)]);
end

scatter(result(:,1), result(:,3),1);
fclose(fid);