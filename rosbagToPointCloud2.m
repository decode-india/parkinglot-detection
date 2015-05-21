% same as one, but using Matlab Robotics System Toolbox
function [pointclouds, colordata] = rosbagToPointCloud2(bagfile, outputFolder)

    bag = rosbag(bagfile);

    cameraDepthTopic = '/camera/depth/image';
    cameraDepthRegisteredTopic = '/camera/depth_registered/points';
    messageStream = select(bag, 'Topic', cameraDepthRegisteredTopic);

    numMessages = messageStream.NumMessages;

    % ----------------------------------------
    %% Nothing
    % for i = 1:numMessages
    for i = 1:1
        pointsBatch = readMessages(messageStream,1:10);
        points = pointsBatch{i};
        points.PreserveStructureOnRead = true;
        pointsXYZ = readXYZ(points);
        pointsRGB = readRGB(points);

        pointCloudFieldNames = {'x', 'y', 'z', 'rgb'};
    end;

    % ----------------------------------------
    %% Metadata
    pointsBatch = readMessages(messageStream,1);
    firstFrame = pointsBatch{1};
    imHeight = firstFrame.Height;
    imWidth = firstFrame.Width;
    channels = firstFrame.PointStep; % number of channels for each pixel

    % ----------------------------------------

    depthFolder = fullfile(outputFolder, 'depth');
    rgbFolder = fullfile(outputFolder, 'rgb');
    makeFolder(depthFolder);
    makeFolder(rgbFolder);
    numFormat = '%05d'; % always 5 digits long

    %% For each timestep, generate a point cloud
    pointclouds = cell(numMessages, 1);
    colordata = cell(numMessages, 1);
    for i = 1:numMessages

        pointsBatch = readMessages(messageStream,i);
        points = pointsBatch{1};
        points.PreserveStructureOnRead = true; % Makes points dimensions imHeight*imWidth*channels
        pointsXYZ = readXYZ(points);
        pointsRGB = readRGB(points);

        depthPath = fullfile(depthFolder, [num2str(i, numFormat) '.mat']);
        save(depthPath, 'pointsXYZ');

        rgbImgPath = fullfile(rgbFolder, [num2str(i, numFormat) '.png']);
        imwrite(pointsRGB, rgbImgPath)

        pointclouds{i} = pointsXYZ;
        colordata{i} = pointsRGB;
    end % for

    pointCloudPath = fullfile(depthFolder, ['pointclouds.mat']);
    save(pointCloudPath, 'pointclouds');
    colordataPath = fullfile(rgbFolder, ['colordata.mat']);
    save(colordataPath, 'colordata');
end % function
