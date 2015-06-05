% ========================================
% Preprocessing
% ========================================

% % ========================================
% % Old Image
% imgFolder = '/home/vgan/code/datasets/gan2015wheelchair/2015-03-22-16-37-50/';
% % imgFolder = '/home/vgan/code/datasets/gan2015wheelchair/2015-03-22-16-41-16/';
% rgbFolder = fullfile(imgFolder, 'rgb');
% depthFolder = fullfile(imgFolder, 'depth');
% % rgbFilePath = fullfile(rgbFolder, '00001.png');
% % depthFilePath = fullfile(depthFolder, '00001.mat');
% rgbFilePath = fullfile(rgbFolder, 'rgb_1.jpg');
% depthFilePath = fullfile(depthFolder, 'depth_1.jpg');
% % rgbFilePath = fullfile(rgbFolder, 'rgb_112.jpg');
% % depthFilePath = fullfile(depthFolder, 'depth_112.jpg');
% imRgb = imread(rgbFilePath);
% imDepth = imread(depthFilePath);
% 
% % ----------------------------------------
% % Flip
% % going from (0,0) in top left to (0,0) in bottom right
% % ----------------------------------------
% isUpsideDown = true; 
% isMirrored = true;
% imDepthFlipped = flipImage(imDepth, isUpsideDown, isMirrored);
% 
% % ----------------------------------------
% % Convert to Point Cloud
% % ----------------------------------------
% [pointcloudRaw, ~] = depthToCloud(imDepthFlipped);

% ========================================
% New Point Cloud

% imgFolder = '/home/vgan/code/datasets/gan2015wheelchair/2015-03-22-16-37-50/';
% imgNumber = '00001';

% imgFolder = '/home/vgan/code/datasets/gan2015wheelchair/2015-03-22-16-41-16/';
% imgNumber = '00111';

% Table - Chair - Chair
% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-17-58.bag';
% imgNumber = '00001';

% Table - Chair - Nothing
% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-27-49.bag';
% imgNumber = '00031';

% Table - Wheelchair - Wheelchair
imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-39-27.bag';
imgNumber = '00001';

% Nothing - Wheelchair - Wheelchair
% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-47-32.bag';
% imgNumber = '00001';


rgbFolder = fullfile(imgFolder, 'rgb');
depthFolder = fullfile(imgFolder, 'depth');
rgbFilePath = fullfile(rgbFolder, [imgNumber '.png']);
depthFilePath = fullfile(depthFolder, [imgNumber '.mat']);

imRgb = imread(rgbFilePath);
variableName = 'pointsXYZ';
load(depthFilePath, variableName);
pointcloudRaw = pointsXYZ;

figure;
subplot(2,1,1)
imshow(imRgb)
title('RGB Image');
subplot(2,1,2)
imshow(pointsXYZ(:,:,3), [])
colormap(parula)
title('Depth Image');

% ----------------------------------------
% Process Point Cloud
% ----------------------------------------
% voxelGridSize = 2; % m
% ransacParams.floorPlaneTolerance = 2; % tolerance in m
% ransacParams.maxInclinationAngle = 30; % in degrees
voxelGridSize = 0.01; % in metres
ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
ransacParams.maxInclinationAngle = 30; % in degrees
[pointCloudRotated, newOrigin] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams);

% ----------------------------------------
% Point Cloud to Ground and Object Map
% ----------------------------------------
% groundMap: the visible ground
% objectMap: what's occupied above ground
gridStepMap = 0.05;    % each pixel represents grisStepMap metres
groundThreshold = 0.1; % in metres
[objectMapHist, groundMapHist, origin] = pointCloudToObjectMap(pointCloudRotated, newOrigin, gridStepMap, groundThreshold);
[mapYSize,mapXSize] = size(objectMapHist);

% ----------------------------------------
% Generate Occupancy Map 
% ----------------------------------------
% Each pixel is either in objectMap, groundMap, or unknownMap
% Apply 0.5 sigma Gaussian blur to get rid of small holes in perception
sigmaWall = 0.7;
objectMap = imgaussfilt(objectMapHist, sigmaWall) ~= 0; % > 1 for noise robustness


sigmaGround = 0.5;
groundMap = imgaussfilt(groundMapHist, sigmaGround) ~= 0; 
groundMap(objectMap) = 0;
% ASUS Xtion Pro has Minimum Depth Range. assume anything within 0.5m is viable.
[XX,YY] = meshgrid(1:mapXSize,1:mapYSize);
distFromOrigin = sqrt((XX-origin(2)).^2 + (YY-origin(1)).^2);
viablePixels = distFromOrigin < 0.5/gridStepMap;
groundMap(viablePixels) = true;

unknownMap = ~groundMap & ~objectMap;
showPlots = true;
if showPlots
    figure;
    map = zeros(size(groundMap,1),size(groundMap,2),3);
    map(:,:,1) = objectMap;
    map(:,:,2) = groundMap & ~objectMap;
    map(:,:,3) = unknownMap;
    imshow(map)
    title('Occupancy Map (Magenta) and Visible Ground Plane (Green)');
end

totalMap = ~groundMap | objectMap; % if obstacle occurs in either

% ========================================
% Convolution
% ========================================



% ----------------------------------------
% Generate Wheelchair
% ----------------------------------------
numAngles = 9;
minAngle = -20;
maxAngle = 20;
angles = linspace(minAngle, maxAngle, numAngles);
% wheelchairsize = [41 31]; % make odd to use centre point as reference
wheelchairsize = [15 11]; % make odd to use centre point as reference
wheelchairShapeAngle = makeWheelchairShape(wheelchairsize, angles);
wheelchairMaps = getWheelChairMaps(wheelchairShapeAngle, mapYSize, mapXSize);

% ----------------------------------------
% Find Feasible Wheelchair Configurations
% ----------------------------------------
feasibleStates = zeros(mapYSize, mapXSize, numAngles);
feasibleStates = findFeasibleStates(groundMap, wheelchairMaps, origin);
% not used. TODO remove
% feasibleStates = zeros(mapYSize, mapXSize, numAngles);
% for i = 1:numAngles
%     normalizedWheelchair = wheelchairShapeAngle(:,:,i) ./sum(sum( wheelchairShapeAngle(:,:,i) ));
%     feasibleStates(:,:,i) = conv2(double(totalMap), normalizedWheelchair, 'same');
% end % for


% ----------------------------------------
% Find Best Wheelchair Configuration
% ----------------------------------------
potentialFunction = zeros(mapYSize, mapXSize, numAngles);
potentialFunction = findPotentialFunction(totalMap, origin, wheelchairMaps, angles, feasibleStates);

% Get Best Potential Function
bestRow = -Inf;
bestCol = -Inf;
bestAngle = -Inf; % degrees
bestMaxPotential = -Inf;
for i = 1:numAngles
    maxPotential = max(max(potentialFunction(:,:,i)));
    [row, col] = find( potentialFunction(:,:,i) >= maxPotential, 1, 'first' );
    if maxPotential > bestMaxPotential
        bestRow = row;
        bestCol = col;
        bestAngle = i;
        bestMaxPotential = maxPotential;
    end
end % for
 
% ----------------------------------------
% Generate Best Wheelchair Configuration on Map
% ----------------------------------------
if bestRow ~= -Inf
    wheelChair = zeros(mapYSize,mapXSize);
    wheelChair(bestRow,bestCol) = 1;
    wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,bestAngle), 'same') ~= 0;
    ( sum(totalMap(wheelChair) == 1) == 0 ) % no collisions
end

% potentialFunction = visibleWeight + obstacleWeight;
% imshow(potentialFunction,[]);

 
% ========================================
% Plot 
% ========================================
% close all;
% figure;
% for i = 1:9
%     subplot(3,3,i)
%     x = zeros(size(feasibleStates,1),size(feasibleStates,2),3);
%     x(:,:,1) = feasibleStates(:,:,i);
%     x(:,:,3) = feasibleStates(:,:,i);
%     imshow(x, [0 30])
%     str = sprintf('Convolved at %d degrees', angles(i));
%     title(str);
% end
% figure;
% titlestringFunc = @(i) sprintf('feasibleStates: %d degrees', angles(i));
% plotConfigurationSpace(feasibleStates, titlestringFunc);



% hold on
% p = pointCloud(pointsT');
% showPointCloud(p);
% showPointCloud(select(p, planeIndicies));


% subplot(2,2,4)
figure
map3 = zeros(mapYSize, mapXSize);
mapAndChair = wheelChair | totalMap;
map3(:,:,1) = mapAndChair;
map3(:,:,2) = wheelChair;
map3(:,:,3) = totalMap;
imshow(map3)
str = sprintf('Chosen Wheelchair Parking Spot, %d to %d degrees', minAngle, maxAngle);
title(str);

figure;
titlestringFunc = @(i) sprintf('desirabilityFunction: %d degrees', angles(i));
plotConfigurationSpace(potentialFunction, titlestringFunc);
