function [pointclouds, colordata] = rosbagToPointCloud(bagfile, outputFolder)

    % ----------------------------------------
    %% Read bag file
    bag = ros.Bag(bagfile);

    depthPointsTopicName = '/camera/depth_registered/points';
    depthPointsTopic = bag.readAll(depthPointsTopicName);

    % ----------------------------------------
    %% Metadata
    firstFrame = depthPointsTopic{1};
    imHeight = firstFrame.height;
    imWidth = firstFrame.width;
    channels = firstFrame.point_step; % number of channels for each pixel

    % ----------------------------------------

    depthFolder = fullfile(outputFolder, 'depth');
    rgbFolder = fullfile(outputFolder, 'rgb');
    makeFolder(depthFolder);
    makeFolder(rgbFolder);
    numFormat = '%05d'; % always 5 digits long

    %% For each timestep, generate a point cloud
    numFrames = length(depthPointsTopic);
    pointclouds = cell(numFrames, 1);
    colordata = cell(numFrames, 1);
    for i = 1:numFrames
        pointData = depthPointsTopic{i}.data;
        [points, imgRGB] = depthRegisteredToPoints(pointData, [imHeight, imWidth, channels]);

        depthPath = fullfile(depthFolder, [num2str(i, numFormat) '.mat']);
        save(depthPath, 'points');

        rgbImgPath = fullfile(rgbFolder, [num2str(i, numFormat) '.png']);
        imwrite(imgRGB, rgbImgPath)

        pointclouds{i} = points;
        colordata{i} = imgRGB;
    end % for




end % function

function [points, imgRGB] = depthRegisteredToPoints(pointData, sizes)
    imHeight = sizes(1);
    imWidth  = sizes(2);
    channels = sizes(3); 
    assert(channels == 32, 'invalid number of channels for depth registered');

    % ----------------------------------------
    %% Massage pointData to imHeight x imWidth x channels matrix
    pointData = reshape(pointData, channels, imWidth, imHeight);
    pointData = permute(pointData,[3 2 1]);

    % ----------------------------------------
    %% Populate these four arrays
    % Channels
    % 1-4:  x single
    % 5-8:  y single
    % 9-12: z single
    % 13-16: 0's (empty)
    % 17: B
    % 18: G
    % 19: R
    % 20-32: 0's (empty)

    xChannels = 1:4;
    yChannels = 5:8;
    zChannels = 9:12;
    rgbChannels = 19:-1:17;

    imgX  = zeros(imHeight,imWidth, 'single');
    imgY  = zeros(imHeight,imWidth, 'single');
    imgZ  = zeros(imHeight,imWidth, 'single');
    imgRGB = zeros(imHeight, imWidth, 3, 'uint8');
    for r = 1:imHeight
    for c = 1:imWidth
        pixelBytes = squeeze(pointData(r,c,:));
        xBytes = pixelBytes(xChannels);
        yBytes = pixelBytes(yChannels);
        zBytes = pixelBytes(zChannels);
        imgX(r,c) = typecast(xBytes, 'single');
        imgY(r,c) = typecast(yBytes, 'single');
        imgZ(r,c) = typecast(zBytes, 'single');
        imgRGB(r,c,:) = pointData(r,c,rgbChannels);
    end
    end

    % ----------------------------------------
    %% Amalgamate
    points = zeros(imHeight,imWidth,3,'double');
    points(:,:,1) = imgX;
    points(:,:,2) = imgY;
    points(:,:,3) = imgZ;
end % function
