function txtwrite(loc_name,A,title,isappend)

% 数据写入函数

if nargin == 2
    title = '';
    isappend = false;
elseif nargin == 3
    isappend = false;
end

if isempty(loc_name)
    loc_name = strcat(date,'.txt');
end
if ~isappend
    fileID = fopen(loc_name,'w');
else
    fileID = fopen(loc_name,'a');
end

if ~isempty(title)
    if ischar(title)
        fprintf(fileID,'%s\r\n',title);
    elseif iscell(title)
        for i = 1:length(title)
            if isempty(title{i})
                continue;
            end
            fprintf(fileID,'%s',title{i});
            if i ~= length(title)
                fprintf(fileID,',');
            end
        end
        fprintf(fileID,'\r\n');
    end
end

size_A = size(A);

if size_A(1) > 1 && size_A(2) > 1
    formats = '';
    for i = 1:size_A(2)
        formats = strcat(formats,'%6.8E');
        if i ~= size_A(2)
            formats = strcat(formats,',');
        else
            formats = strcat(formats,' \r\n');
        end
    end
    fprintf(fileID,formats,A');
end

fclose(fileID);




