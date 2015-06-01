% Input: a filepath to an image folder and an extension (eg. '.png')
% Output:   filename: a cell array of all the filenames of that extension
%           fullFilePath: filenames appended to imageFolder
function [filename, fullFilePath] = filenamesInFolder(imageFolder, imageExt)
    assert(imageExt(1) == '.', 'Not a valid file extension')

    directory = dir(fullfile(imageFolder, ['*' imageExt]));

    first_frame = 1;
    last_frame = length(directory); % last frame

    fullFilePath = cell(last_frame - first_frame + 1,1);
    filename     = cell(last_frame - first_frame + 1,1);
    for i = first_frame:last_frame 
        filename{i - first_frame + 1}     = directory(i).name;
        fullFilePath{i - first_frame + 1} = fullfile(imageFolder, directory(i).name);
    end

end % function
