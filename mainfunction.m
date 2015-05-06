% ========================================
% Preprocessing
% ========================================
imgFolder = '/home/vgan/code/datasets/gan2015wheelchair/2015-03-22-16-37-50/';
rgbFolder = fullfile(imgFolder, 'rgb');
depthFolder = fullfile(imgFolder, 'depth');

rgbFilename = fullfile(rgbFolder, 'rgb_1.jpg');
depthFilename = fullfile(depthFolder, 'depth_1.jpg');

% ----------------------------------------
% Flip
% going from (0,0) in top left to (0,0) in bottom right
% ----------------------------------------
imRgb = imread(rgbFilename);
imDepth = imread(depthFilename);

% imRgbFlipped = flipImage(imRgb, isUpsideDown, isMirrored);

[occupancyMap, groundMap, origin] = depthToOccupancyMap(imDepth);

[ySize,xSize] = size(occupancyMap);

% ASUS Xtion Pro has Minimum Depth Range. assume anything within 0.5m is viable.
% [XX,YY] = meshgrid(1:xSize,1:ySize);
% distFromOrigin = sqrt((XX-origin(2)).^2 + (YY-origin(1)).^2);
% radius = 0.5;
% viablePixels = distFromOrigin < 50/gridStepMap;
% groundMap(viablePixels) = true;

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
wheelchairsize = [41 31]; % make odd to use centre point as reference
wheelchairShapeAngle = makeWheelchairShape(wheelchairsize, angles);

% ----------------------------------------
% Generate Occupancy Map + Wheelchair
% ----------------------------------------
% Apply 0.5 sigma Gaussian blur to get rid of small holes in perception
visibleMap = imgaussfilt(groundMap) == 0; 
wallMap = imgaussfilt(occupancyMap) ~= 0; % >1 for noise robustness
totalMap = visibleMap | wallMap; % if obstacle occurs in either

feasibleStates = zeros(ySize, xSize, numAngles);
for i = 1:numAngles
    feasibleStates(:,:,i) = conv2(double(totalMap), wheelchairShapeAngle(:,:,i), 'same');
end % for

% ----------------------------------------
% Find Best Wheelchair Configuration
% ----------------------------------------
% numAngles = size(wheelchairShapeAngle, 3);
bestRow = -Inf;
bestCol = -Inf;
bestAngle = -Inf; % degrees
potentialFunction = zeros(ySize, xSize, numAngles);
for i = 1:numAngles
    potentialFunction(:,:,i) = findPotentialFunction(totalMap, origin, wheelchairShapeAngle(:,:,i));
    maxPotential = max(max(potentialFunction(:,:,i)));
    [row, col] = find( potentialFunction(:,:,i) >= maxPotential, 1, 'first' );
    if maxPotential > bestMaxPotential
        bestRow = row;
        bestCol = col;
        bestAngle = i;
    end
end % for

% ----------------------------------------
% Generate Best Wheelchair Configuration on Map
% ----------------------------------------
wheelChair = zeros(ySize,xSize);
wheelChair(bestRow,bestCol) = 1;
wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,bestAngle), 'same') ~= 0;
( sum(totalMap(wheelChair) == 1) == 0 ) % no collisions


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
figure;
titlestringFunc = @(i) sprintf('feasibleStates: %d degrees', angles(i));
plotConfigurationSpace(feasibleStates, titlestringFunc);

figure;
subplot(2,2,1)
imshow(imRgb)
title('RGB Image');
subplot(2,2,2)
imshow(imDepth)
title('Depth Image');

subplot(2,2,3)
showPointCloud(pointCloudVoxeled);
colormap(parula)
title('Voxelized Point Cloud with Ground Plane');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;

% Plot 
p = pointCloudVoxeled;
a = updatedPlane(1);
b = updatedPlane(2);
c = updatedPlane(3);
d = updatedPlane(4);
numTicks = 100;
xScale = linspace(p.XLimits(1),p.XLimits(2), numTicks);
yScale = linspace(p.YLimits(1),p.YLimits(2), numTicks);
zScale = linspace(p.ZLimits(1),p.ZLimits(2), numTicks);
[xx,yy,zz] = meshgrid(xScale, yScale, zScale);
isosurface(xx, yy, zz, a*xx+b*yy+c*zz+d, 0)

% Plot Point Cluod
subplot(2,2,4)
showPointCloud(pointCloudRotated);
colormap(parula)
title('Rotated relative to Ground');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;

% hold on
% p = pointCloud(pointsT');
% showPointCloud(p);
% showPointCloud(select(p, planeIndicies));

figure;
subplot(2,2,1)
map = zeros(size(groundMap,1),size(groundMap,2),3);
map(:,:,1) = groundMap;
map(:,:,2) = occupancyMap;
map(:,:,3) = occupancyMap;
imshow(map)
title('Occupancy Map (Cyan) and Visible Ground Plane (Red)');

subplot(2,2,2)
map2 = zeros(size(groundMap,1),size(groundMap,2),3);
map2(:,:,1) = totalMap;
map2(:,:,3) = totalMap;
imshow(map2)
title('Permissible Configurations');

subplot(2,2,3)
imshow(potentialFunction(:,:,bestAngle), [], 'Colormap', parula)
title('Potential Function for Chosen Angle (Red is Better)');


subplot(2,2,4)
map3 = zeros(size(groundMap,1),size(groundMap,2),3);
mapAndChair = wheelChair | totalMap;
map3(:,:,1) = mapAndChair;
map3(:,:,2) = wheelChair;
map3(:,:,3) = totalMap;
imshow(map3)
str = sprintf('Chosen Wheelchair Parking Spot, %d to %d degrees', minAngle, maxAngle);
title(str);

figure;
titlestringFunc = @(i) sprintf('potentialFunction: %d degrees', angles(i));
plotConfigurationSpace(potentialFunction, titlestringFunc);
