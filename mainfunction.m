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

% ========================================
% Generate ground/object maps 
% ========================================
% ----------------------------------------
% Process Point Cloud
% ----------------------------------------
% voxelGridSize = 2; % m
% ransacParams.floorPlaneTolerance = 2; % tolerance in m
% ransacParams.maxInclinationAngle = 30; % in degrees
voxelGridSize = 0.01; % in metres
ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
ransacParams.maxInclinationAngle = 30; % in degrees
[pointCloudRotated, originPointcloud] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams);

% ----------------------------------------
% Point Cloud to Ground and Object Map
% ----------------------------------------
% groundMap: the visible ground
% objectMap: what's occupied above ground
MetresPerMapUnit = 0.05;    % each pixel represents grisStepMap metres
groundThreshold = 0.1; % in metres
[objectMapHist, groundMapHist, origin] = pointCloudToObjectMap(pointCloudRotated, originPointcloud, MetresPerMapUnit, groundThreshold);
[mapYSize,mapXSize] = size(objectMapHist);

% ----------------------------------------
% Generate Occupancy Map 
% ----------------------------------------
% Each pixel is either in objectMap, groundMap, or unknownMap
% Apply 0.5 sigma Gaussian blur to get rid of small holes in perception
% TODO morphological dilation?
sigmaWall = 0.7;
objectMap = imgaussfilt(objectMapHist, sigmaWall) ~= 0; % > 1 for noise robustness

sigmaGround = 0.5;
groundMap = imgaussfilt(groundMapHist, sigmaGround) ~= 0; 
groundMap(objectMap) = 0;

% ASUS Xtion Pro has Minimum Depth Range. assume anything within 0.5m is viable.
originDistMetres = 0.5;
originDistMapUnits = originDistMetres/MetresPerMapUnit;
viablePixels = circleAroundPoint(origin, mapYSize, mapXSize, originDistMapUnits);
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
totalMap = ~groundMap; % 0 if ground, 1 if nonground

% ========================================
% Find a Good Wheelchair Configuration
% ========================================
% ----------------------------------------
% Generate Wheelchair - TODO speed up
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

% ----------------------------------------
% Find Wheelchair Configuration Potential Function
% ----------------------------------------
potentialFunction = zeros(mapYSize, mapXSize, numAngles);
potentialFunction = findPotentialFunction(totalMap, origin, wheelchairMaps, angles, feasibleStates);

if showPlots
    figure;
    titlestringFunc = @(i) sprintf('desirabilityFunction: %d degrees', angles(i));
    plotConfigurationSpace(potentialFunction, titlestringFunc);
end % if showPlots

% ----------------------------------------
% Choose Best Wheelchair Configuration on Map
% ----------------------------------------
[bestState, bestPotential] = chooseBestState(potentialFunction);
noFeasibleStates = isempty(bestState);
if ~noFeasibleStates 
    if showPlots
        plotState(bestState, wheelchairMaps, totalMap);
    end % if showPlots
else
    disp('There are no feasible states =(');
end
