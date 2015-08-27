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

% Wall - Wall - Wall
% imgFolder = '/home/vgan/code/datasets/gan2015wheelchair/2015-03-22-16-37-50/';
% imgNumber = '00031';
% imgNumber = '00001';

% imgFolder = '/home/vgan/code/datasets/gan2015wheelchair/2015-03-22-16-41-16/';
% imgNumber = '00111';

imgFolder = '/media/vgan/stashpile1/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-17-58.bag';
imgNumber = '00001'; % Table - Chair - Chair
% imgNumber = '00128'; % Table - Chair - Chair Close
% imgNumber = '00220'; % Table - Chair - Chair Closer
% imgNumber = '00522'; % Table - Chair - Chair Off-angle
% imgNumber = '00568'; % Open Space

% Table - Chair - Nothing
% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-28-59.bag';
% imgNumber = '00098'; % cabinet and chair corner.

% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-39-27.bag';
% imgNumber = '00001';

% Table - Wheelchair - Wheelchair
% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-39-27.bag';
% imgNumber = '00001';

% Nothing - Wheelchair - Wheelchair
% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-47-32.bag';
% imgNumber = '00001'; % Nothing - Wheelchair - Wheelchair
% imgNumber = '00086'; % large empty space

% imgFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables/matlab_extract/2015-05-28-23-42-24.bag';
% imgNumber = '00001'; % Table - WC - WC Large parking space

% Process input files
rgbFolder = fullfile(imgFolder, 'rgb');
depthFolder = fullfile(imgFolder, 'depth');
rgbFilePath = fullfile(rgbFolder, [imgNumber '.png']);
depthFilePath = fullfile(depthFolder, [imgNumber '.mat']);

imRgb = imread(rgbFilePath);
variableName = 'pointsXYZ';
load(depthFilePath, variableName);
pointcloudRaw = pointsXYZ;

% Output Folder
[~, bagfilename, ~] = fileparts(imgFolder);
outputFolder = '/home/vgan/code/experiments/parkinglot-detection-output';
dateFolder = '20150826';
saveFolder = fullfile(outputFolder, dateFolder, [bagfilename '_' imgNumber]);
makeFolder(saveFolder);

showPlots = true;
if showPlots
    figure;
    subplot(1,2,1)
    imshow(imRgb)
    title('RGB Image');
    imwrite(imRgb, fullfile(saveFolder, 'imRgb.png'));

    subplot(1,2,2)
    imDepth = pointsXYZ(:,:,3);
    imshow(imDepth, [])
    colormap(parula)
    title('Depth Image');
    imwrite(imDepth, parula, fullfile(saveFolder, 'imDepth.png'));
    savefig(fullfile(saveFolder, 'rgb-depth-image.fig'));
end % if showPlots

% ========================================
% Generate ground/object maps 
% ========================================
% ----------------------------------------
% Process Point Cloud
% ----------------------------------------
voxelGridSize = 0.01; % in metres
ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
ransacParams.maxInclinationAngle = 30; % in degrees
[pointCloudRotated, originPointcloud] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams);

% ----------------------------------------
% Point Cloud to Ground and Object Map
% ----------------------------------------
% groundMap: the visible ground
% objectMap: what's occupied above ground
metresPerMapUnit = 0.05;    % each pixel represents grisStepMap metres
groundThreshold = 0.1; % in metres
[objectMapHist, groundMapHist, origin] = pointCloudToObjectMap(pointCloudRotated, originPointcloud, metresPerMapUnit, groundThreshold);
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
groundMap(objectMap) = false;

% ASUS Xtion Pro has Minimum Depth Range. assume anything within 0.5m is viable.
originDistMetres = 1.0;
originDistMapUnits = originDistMetres/metresPerMapUnit;
viablePixels = circleAroundPoint(origin, mapYSize, mapXSize, originDistMapUnits);
groundMap(viablePixels) = true;
groundMap(objectMap) = false;

overlaps = groundMap & objectMap;
assert( sum(overlaps(:)) == 0, 'groundMap and objectMap are not mutually exclusive');
unknownMap = ~groundMap & ~objectMap;

if showPlots
    figure;
    map = zeros(size(groundMap,1),size(groundMap,2),3);
    map(:,:,1) = objectMap;
    map(:,:,2) = groundMap;
    map(:,:,3) = unknownMap;
    map(origin(1),origin(2),:) = 0;
    imshow(map)
    title('Object Map (Red), Ground Map (Green), Unknown Map (Blue)');
    imwrite(map, fullfile(saveFolder, 'objectGroundMap.png'));
end % if showPlots

% ========================================
% Find a Good Wheelchair Configuration
% ========================================
% ----------------------------------------
% Generate Wheelchair - TODO speed up
% ----------------------------------------
numAngles = 9;
minAngle  = -5; % degrees
maxAngle  =  5; % degrees
angles = linspace(minAngle, maxAngle, numAngles);
% wheelchairsize = [23 23]; % make odd to use centre point as reference
wheelchairsize = [15 11]; % make odd to use centre point as reference
wheelchairShapeAngle = makeWheelchairShape(wheelchairsize, angles);

% ----------------------------------------
% Find Feasible Wheelchair Configurations
% ----------------------------------------
feasibleStates = zeros(mapYSize, mapXSize, numAngles); % TODO Remove
feasibleStates = findFeasibleStates(groundMap, wheelchairShapeAngle, origin);

% ----------------------------------------
% Find Wheelchair Configuration Potential Function
% ----------------------------------------
wheelchairMaps = getWheelChairMaps(wheelchairShapeAngle, mapYSize, mapXSize);
statePotentials = findPotentialFunction(groundMap, objectMap, wheelchairMaps, feasibleStates);

% halfPaddingHeight = floor( (numRows - wheelchairsize(1))/2 - 1 ) + 1;
% halfPaddingWidth  = floor( (numCols - wheelchairsize(2))/2 - 1 ) + 1;
halfPaddingHeight = numRows;
halfPaddingWidth  = numCols;
wheelchairShapeAngleBig = makeWheelchairShape(wheelchairsize, angles, [halfPaddingHeight, halfPaddingWidth]);

if showPlots
    figure;
    titlestringFunc = @(i) sprintf('desirabilityFunction: %d degrees', angles(i));
    plotConfigurationSpace(statePotentials, titlestringFunc);
    savefig(fullfile(saveFolder, 'potential-function.fig'));
end % if showPlots

% ----------------------------------------
% Choose Best Wheelchair Configuration on Map
% ----------------------------------------
[chosenState, bestPotential] = chooseBestState(statePotentials);
noFeasibleStates = isempty(chosenState);
if ~noFeasibleStates 
    if showPlots
        plotState(chosenState, wheelchairMaps, ~groundMap);
        savefig(fullfile(saveFolder, 'final-position-map.fig'));
    end % if showPlots
else
    disp('There are no feasible states =(');
end

% ----------------------------------------
% Plot in Point Cloud
% ----------------------------------------
if showPlots
    figure;

    % Plot Pointcloud
    titleString = 'Rotated relative to Ground';
    plotPointCloud(pointCloudRotated, titleString);
    hold on;

    % Plot Wheelchair
    green = [0 1 0];
    wheelChair = wheelchairMaps{chosenState(1), chosenState(2), chosenState(3)};
    wheelChairDepths = 0:metresPerMapUnit:0.8;
    plotMapInPointcloud(wheelChair, pointCloudRotated, metresPerMapUnit, green, wheelChairDepths);

    % Plot Ground and Object Map
    groundMapDepth = -1.0;
    blue = [0 0 1];
    plotMapInPointcloud(groundMap, pointCloudRotated, metresPerMapUnit, blue, groundMapDepth);
    red = [1 0 0];
    plotMapInPointcloud(objectMap, pointCloudRotated, metresPerMapUnit, red, groundMapDepth);

    % Plot Feasible States
    feasibleStatesDepth = -2.0;
    green = [0 1 0];
    plotMapInPointcloud(feasibleStates(:,:,chosenState(3)), pointCloudRotated, metresPerMapUnit, green, feasibleStatesDepth);

    savefig(fullfile(saveFolder, 'final-position.fig'));
end % if showPlots
