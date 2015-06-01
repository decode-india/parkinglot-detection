% same as one, but using Matlab Robotics System Toolbox
% Input:
%   bagFilePath: full file path of .bag file
%   outputFolder (optional): folder to save output (takes a while)
% Output:
%   pointclouds: struct of imHeight x imWidth x 3 (x, y, z) arrays, single 0 to ~10 (metres).
%   colordata: struct of imHeight x imWidth x 3 (r, g, b) arrays, double 0 to 1.
function [pointclouds, colordata] = rosbagToPointCloud2(bagFilePath, outputFolder)

    switch nargin
        case 1
            saveFiles = false;
        case 2
            saveFiles = true;
    end % switch

    % ----------------------------------------
    %% Read bag file
    bag = rosbag(bagFilePath);
    cameraDepthRegisteredTopic = '/camera/depth_registered/points';
    messageStream = select(bag, 'Topic', cameraDepthRegisteredTopic);
    numMessages = messageStream.NumMessages;

    % ----------------------------------------
    %% Metadata
    pointsBatch = readMessages(messageStream,1);
    firstFrame = pointsBatch{1};
    imHeight = firstFrame.Height;
    imWidth = firstFrame.Width;
    channels = firstFrame.PointStep; % number of channels for each pixel

    % ----------------------------------------
    if saveFiles
        depthFolder = fullfile(outputFolder, 'depth');
        rgbFolder = fullfile(outputFolder, 'rgb');
        makeFolder(depthFolder);
        makeFolder(rgbFolder);
        numFormat = '%05d'; % always 5 digits long
    end

    %% For each timestep, generate a point cloud
    pointclouds = cell(numMessages, 1);
    colordata = cell(numMessages, 1);
    for i = 1:numMessages

        pointsBatch = readMessages(messageStream,i);
        points = pointsBatch{1}; % pointsBatch is 1x1 cell array since readMessages only reads ith message
        points.PreserveStructureOnRead = true; % Makes points dimensions imHeight*imWidth*channels
        pointsXYZ = readXYZ(points);
        pointsRGB = readRGB(points);

        if saveFiles
            depthPath = fullfile(depthFolder, [num2str(i, numFormat) '.mat']);
            save(depthPath, 'pointsXYZ');

            rgbImgPath = fullfile(rgbFolder, [num2str(i, numFormat) '.png']);
            imwrite(pointsRGB, rgbImgPath);
        end

        pointclouds{i} = pointsXYZ;
        colordata{i} = pointsRGB;
    end % for

    if saveFiles
        pointCloudPath = fullfile(depthFolder, ['pointclouds.mat']);
        save(pointCloudPath, 'pointclouds', '-v7.3');
        colordataPath = fullfile(rgbFolder, ['colordata.mat']);
        save(colordataPath, 'colordata', '-v7.3');
    end
end % function
