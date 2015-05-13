% Makes a folder if it doesn't already exist
function makeFolder(folderName)
    if ~isdir(folderName)
        mkdir(folderName); 
    end
end % function

