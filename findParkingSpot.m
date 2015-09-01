function [chosenStateWorldX, chosenStateWorldY, R_OpticToGround, T_OpticToGround] = findParkingSpot(pointcloudRaw, saveFolder)
    makeFolder(saveFolder);

    % ----------------------------------------
    % Show RGBD Images
    % ----------------------------------------
    showPlots = true;
    if showPlots
        figure
        imDepth = pointcloudRaw(:,:,3);
        imshow(imDepth, [])
        colormap(parula)
        title('Depth Image');
        imwrite(imDepth, parula, fullfile(saveFolder, 'imDepth.png'));
        savefig(fullfile(saveFolder, 'rgb-depth-image.fig'));
    end % if showPlots

    % ----------------------------------------
    % Process Point Cloud
    % ----------------------------------------
    voxelGridSize = 0.05; % in metres
    ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
    ransacParams.maxInclinationAngle = 30; % in degrees
    [pointCloudRotated, originPointcloud, R_OpticToGround, T_OpticToGround] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams);

    % ----------------------------------------
    % Point Cloud to Ground and Object Map
    % ----------------------------------------
    % groundMap: the visible ground
    % objectMap: what's occupied above ground
    metresPerMapUnit = 0.05; % each pixel represents metresPerMapUnit metres
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
    feasibleStates = findFeasibleStates(groundMap, wheelchairShapeAngle, origin);

    % ----------------------------------------
    % Find Wheelchair Configuration Potential Function
    % ----------------------------------------
    halfPaddingHeight = (mapYSize - 1) - (wheelchairsize(1)-1)/2;
    halfPaddingWidth = (mapXSize - 1) - (wheelchairsize(2)-1)/2;
    wheelchairShapeAngleBig = makeWheelchairShape(wheelchairsize, angles, [halfPaddingHeight, halfPaddingWidth]);
    statePotentials = findPotentialFunction(objectMap, wheelchairShapeAngleBig, feasibleStates);

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
            plotState(chosenState, wheelchairShapeAngle, ~groundMap);
            savefig(fullfile(saveFolder, 'final-position-map.fig'));
        end % if showPlots
    else
        disp('There are no feasible states =(');
    end

    chosenStateMap = false(mapYSize, mapXSize); 
    chosenStateMap(chosenState(1), chosenState(2)) = true;
    xScale = pointCloudRotated.XLimits(1):metresPerMapUnit:pointCloudRotated.XLimits(2);
    yScale = pointCloudRotated.YLimits(1):metresPerMapUnit:pointCloudRotated.YLimits(2);
    [chosenStateWorldX, chosenStateWorldY] = pixelsToPoints(chosenStateMap, xScale, yScale);
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

        wheelChair = zeros(mapYSize, mapXSize);
        wheelChair(chosenState(1),chosenState(2)) = 1;
        wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,chosenState(3)), 'same');
        wheelChair = wheelChair ~= 0;
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

end % function
