
numAngles = 1;
% minAngle  = -5; % degrees
% maxAngle  =  5; % degrees
minAngle  = 0; % degrees
maxAngle  = 0; % degrees
angles = linspace(minAngle, maxAngle, numAngles);

[mapYSize,mapXSize] = size(objectMap);

% wheelchairsize = [23 23]; % make odd to use centre point as reference
% wheelchairsize = [15 11]; % [y x] make odd to use centre point as reference
wheelchairsize = [1 1]; % [y x] make odd to use centre point as reference
wheelchairShapeAngle = makeWheelchairShape(wheelchairsize, angles);
feasibleStates = true(mapYSize, mapXSize, numAngles); % TODO Remove

numStates = 2.^[1:23];
numSizes = length(numStates);
objectMapResized = cell(1,numSizes);
numRealStates = zeros(1, numSizes);
for i = 1:numSizes
    numPixels = (numStates(i) / numAngles);
    newHeight =  ceil( sqrt(numPixels / (mapXSize*mapYSize)) * mapYSize );
    objectMapResized{i} = imresize(objectMap, [newHeight NaN]);
    numRealStates(i) = size(objectMapResized{i},1)*size(objectMapResized{i},2)*numAngles;
end

tic
timeUnOptimized      = zeros(1, numSizes);
timeOptimized        = zeros(1, numSizes);
timeUnOptimizedNoGpu = zeros(1, numSizes);
timeOptimizedNoGpu   = zeros(1, numSizes);
for i = 1:numSizes
    [mapYSizeNew,mapXSizeNew] = size(objectMapResized{i});
    halfPaddingHeight = (mapYSizeNew - 1) - (wheelchairsize(1)-1)/2;
    halfPaddingWidth = (mapXSizeNew - 1) - (wheelchairsize(2)-1)/2;
    wheelchairShapeAngleBig = makeWheelchairShape(wheelchairsize, angles, [halfPaddingHeight, halfPaddingWidth]);
    feasibleStates = true(mapYSizeNew, mapXSizeNew, numAngles); % TODO Remove

    startOffset = toc;
    statePotentials = findPotentialFunction(objectMapResized{i}, wheelchairShapeAngleBig, feasibleStates);
    elapsedTime = toc;
    timeUnOptimized(i) = elapsedTime - startOffset

    startOffset = toc;
    statePotentials = findPotentialFunction2(wheelchairShapeAngleBig, objectMapResized{i}, feasibleStates);
    elapsedTime = toc;
    timeOptimized(i) = elapsedTime - startOffset

    startOffset = toc;
    statePotentials = findPotentialFunctionNoGpu(objectMapResized{i}, wheelchairShapeAngleBig, feasibleStates);
    elapsedTime = toc;
    timeUnOptimizedNoGpu(i) = elapsedTime - startOffset

    startOffset = toc;
    statePotentials = findPotentialFunction2NoGpu(wheelchairShapeAngleBig, objectMapResized{i}, feasibleStates);
    elapsedTime = toc;
    timeOptimizedNoGpu(i) = elapsedTime - startOffset

    figure(1)
    semilogx(numRealStates, timeUnOptimized, numRealStates, timeOptimized, numRealStates, timeUnOptimizedNoGpu, numRealStates, timeOptimizedNoGpu);
    legend('Unoptimized, GPU', 'Optimized, GPU', 'Unoptimized, CPU', 'Optimized, CPU')
    drawnow
end

