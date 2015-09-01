% Find the desirability for each configuration
function [measureAll] = findPotentialFunction(objectMap, wheelchairShapeAngleBig, feasibleStates)
    [numRows,numCols] = size(objectMap);
    % wheelchairMaps = getWheelChairMaps(wheelchairShapeAngleBig, numRows, numCols);
    numAngles = size(wheelchairShapeAngleBig, 3);

    measureAll = zeros(numRows,numCols,numAngles);
    for thetaIdx = 1:numAngles
    for r = 1:numRows
    for c = 1:numCols
        if ~feasibleStates(r,c,thetaIdx)
            measureAll(r,c,thetaIdx) = -Inf;
        else
            [rLow, rHigh] = getIndicies(r, numRows);
            [cLow, cHigh] = getIndicies(c, numCols);
            wheelChair = wheelchairShapeAngleBig(rLow:rHigh, cLow:cHigh, thetaIdx);
            % wheelChair = wheelchairMaps{r,c,thetaIdx};
            % [measureAll(r,c,thetaIdx), distanceTransform] = minDistBetweenWheelchairAndObstacles(wheelChair, totalMap);
            [measureAll(r,c,thetaIdx), distanceTransform] = sumSquaredClosestDistance2(wheelChair, objectMap);
            % measureAll(r,c,thetaIdx) = sumSquaredClosestDistance(wheelChair, totalMap);
        end % if
    end % for
    end % for
    end % for

end % function

function measure = computePotential(wheelChair, totalMap, x, y, theta) 
    % [numRows,numCols] = size(totalMap);
    % 
    % wheelChair = zeros(numRows,numCols);
    % wheelChair(y,x) = 1;
    % wheelChair = conv2(wheelChair, wheelchairShape, 'same');
    % wheelChair = wheelChair ~= 0;

    % figure
    % imshow(wheelChair);

    % measure = sumSquaredClosestDistance(wheelChair, totalMap);
    % [measure, distanceTransform] = sumSquaredClosestDistanceFlat(wheelChair, totalMap);
    [measure, distanceTransform] = minDistBetweenWheelchairAndObstacles(wheelChair, totalMap);

    % [measure, distanceTransform] = minSideDistBetweenWheelchairAndObstacles(wheelChair, totalMap);
    % [measure2, distanceTransform2] = minFrontDistBetweenWheelchairAndObstacles(wheelChair, totalMap);
    % measure = measure + measure2;
    
    % [measure, distanceTransform] = minOrientedSideDist(wheelChair, totalMap, theta);
    % [measure2, distanceTransform2] = minOrientedFrontDist(wheelChair, totalMap, theta);
    % measure = measure + measure2;

    % pointsToPlot = (x == 70 & y == 50) | (x == 79 & y == 53);
    % if pointsToPlot
    %     figure
    %     subplot(1,2,1)
    %     imshow(distanceTransform, [0 30], 'Colormap', parula);
    %     str = sprintf('Potential Function for Wheelchair (x,y): (%d, %d)', x, y);
    %     title(str);
    %     subplot(1,2,2)
    %     imshow(totalMap, 'Colormap', parula);
    %     title('Obstacle Map');
    % end % if
end % function

% Straight Quadratic
function measure = sumSquaredClosestDistance(wheelChair, totalMap)
    mapAndChair = wheelChair | totalMap;
    mapAndChair = gpuArray(mapAndChair);
    % figure
    % imshow(mapAndChair);

    distanceTransform = bwdist(mapAndChair).^2; % distance to closest edge, square to bias large gaps
    % figure
    % imshow(distanceTransform, [], 'Colormap', jet(255));

    measure = sum(distanceTransform(:));
    measure = double(gather(measure));
end

% Straight Quadratic with a constant at a small value: _/ or minima \_/
function [measure, distanceTransform] = sumSquaredClosestDistanceFlat(wheelChair, totalMap)
    mapAndChair = wheelChair | totalMap;
    mapAndChair = gpuArray(mapAndChair);
    distanceTransform = bwdist(mapAndChair).^2; % distance to closest edge, square to bias large gaps


    threshold = 8; % pixels to wall
    isLessThanThreshold = distanceTransform > 0 & distanceTransform <= threshold.^2;
    constValue = 1;
    distanceTransform(isLessThanThreshold) = constValue; % _/
    % distanceTransform(isLessThanThreshold) = threshold^2 - distanceTransform(isLessThanThreshold); % \_/ 

    measure = sum(distanceTransform(:));
    measure = double(gather(measure));

    distanceTransform = double(gather(distanceTransform));
    % figure
    % imshow(distanceTransform, [], 'Colormap', parula);
end

function [measure, distanceTransform] = sumSquaredClosestDistance2(wheelChair, objectMap)
    mapAndChair = wheelChair | objectMap;
    mapAndChair = gpuArray(mapAndChair);
    % figure
    % imshow(mapAndChair);

    distanceTransform = bwdist(mapAndChair).^2; % distance to closest edge, square to bias large gaps
    % figure
    % imshow(distanceTransform, [], 'Colormap', jet(255));

    % normalize by number of pixels TODO
    measure = sum(distanceTransform(:)) / nnz(distanceTransform);
    measure = double(gather(measure));
end % function

% wheelChair: nxm array, ones with wheelchair position, 0 without
% totalMap: nxm array, ones with obstacle position, 0 without
function [measure, distanceTransform] = minDistBetweenWheelchairAndObstacles(wheelChair, totalMap)
    % figure
    % imshow(wheelChair);
    % figure
    % imshow(totalMap);

    % get wheelchair perimeter
    totalMap = gpuArray(totalMap);
    distanceTransformGpu = bwdist(totalMap); % distance to closest obstacle

    % only take wheelchair perimeter values
    wheelChairPerimeter = bwperim(wheelChair,8);
    wheelChairPerimeter = gpuArray(wheelChairPerimeter);
    
    distanceTransformGpu = distanceTransformGpu .* wheelChairPerimeter;
    % figure
    % imshow(distanceTransformGpu, [], 'Colormap', parula);

    % Pass (distanceTransformGpu) through local minima function
    threshold = 8; % pixels to wall
    isLessThanThreshold = distanceTransformGpu > 0 & distanceTransformGpu <= threshold.^2;
    constValue = 1;
    % distanceTransformGpu(isLessThanThreshold) = constValue; % _/
    distanceTransformGpu(isLessThanThreshold) = threshold^2 - distanceTransformGpu(isLessThanThreshold); % \_/ 

    % measure = sum(distanceTransformGpu(:));
    measure = min(distanceTransformGpu(wheelChairPerimeter));
    measure = double(gather(measure));

    distanceTransform = double(gather(distanceTransformGpu));
end % function

% These two are 'max side distances, min back distance'
function [measure, visualize] = minSideDistBetweenWheelchairAndObstacles(wheelChair, totalMap)

    wheelChairPerimeter = bwperim(wheelChair,8);
    numPerimeterElements = sum(wheelChairPerimeter(:));

    [rows, cols] = find(wheelChairPerimeter);
    closestSideWall = zeros(numPerimeterElements,1);
    for i = 1:numPerimeterElements
        r = rows(i);
        c = cols(i);
        currentRow = totalMap(r,:);
        leftWall = find(currentRow(1:c-1),1,'last');
        rightWall = find(currentRow(c:end),1,'first') + (c - 1); % keep relative to totalMap
        leftWallDist = c - leftWall;
        rightWallDist = rightWall - c;
        closestSideWall(i) = min(leftWallDist, rightWallDist);
    end % for

    measure = min(closestSideWall(:));

    visualize = double(totalMap);
    visualize(wheelChairPerimeter) = closestSideWall;
end % function
function [measure, visualize] = minFrontDistBetweenWheelchairAndObstacles(wheelChair, totalMap)

    wheelChairPerimeter = bwperim(wheelChair,8);
    numPerimeterElements = sum(wheelChairPerimeter(:));

    [rows, cols] = find(wheelChairPerimeter);
    distToFrontWall = zeros(numPerimeterElements,1);
    for i = 1:numPerimeterElements
        r = rows(i);
        c = cols(i);
        currentRow = totalMap(:,c);
        backWall = find(currentRow(1:r-1), 1, 'last');
        frontWall = find(currentRow(r:end), 1, 'first') + (r - 1); % keep relative to totalMap
        backWallDist = r - backWall;
        frontWallDist = frontWall - r;
        distToFrontWall(i) = frontWallDist;
    end % for

    measure = -1 * min(distToFrontWall); % negative to penalize far away dists

    visualize = double(totalMap);
    visualize(wheelChairPerimeter) = distToFrontWall;
end % function

% miniumum distnace between chair and obstacles, form chair's orientation
function [measure, visualize] = minOrientedFrontDist(wheelChair, totalMap, angle)

    wheelChairPerimeter = bwperim(wheelChair,8);
    numPerimeterElements = sum(wheelChairPerimeter(:));
    [ys, xs] = find(wheelChairPerimeter);
    
    [mapHeight, mapWidth] = size(totalMap);

    distFromRear = zeros(numPerimeterElements,1);
    for i = 1:numPerimeterElements
        y = ys(i);
        x = xs(i);
        linemap = rasterlineFromPoint(y,x,angle,mapHeight,mapWidth);

        % All obstacle positions along the line oriented to the front
        lineObstacles = linemap & totalMap;

        [side1Dist, side2Dist] = closestDistsToPoint(x, y, lineObstacles);

        % side1Dist corresponds to the point far from camera
        distFromRear(i) = side1Dist; % TODO Check
    end % for

    measure = -1 * min(distFromRear); % negative to penalize far away dists

    visualize = double(totalMap);
    visualize(wheelChairPerimeter) = distFromRear;
end % function

% miniumum distnace between chair and obstacles, form chair's orientation
function [measure, visualize] = minOrientedSideDist(wheelChair, totalMap, angle)

    wheelChairPerimeter = bwperim(wheelChair,8);
    numPerimeterElements = sum(wheelChairPerimeter(:));
    [ys, xs] = find(wheelChairPerimeter);
    
    [mapHeight, mapWidth] = size(totalMap);

    distFromRear = zeros(numPerimeterElements,1);
    for i = 1:numPerimeterElements
        y = ys(i);
        x = xs(i);
        
        perpendicularAngle = 90 - angle;
        linemap = rasterlineFromPoint(y,x,perpendicularAngle,mapHeight,mapWidth);

        % All obstacle positions along the line oriented to the front
        lineObstacles = linemap & totalMap;


        [side1Dist, side2Dist] = closestDistsToPoint(x, y, lineObstacles);

        closestSideWall(i) = min(side1Dist, side2Dist);
    end % for

    measure = 1 * min(closestSideWall); % positive to penalize close dists

    visualize = double(totalMap);
    visualize(wheelChairPerimeter) = closestSideWall;
end % function

function [side1Dist, side2Dist] = closestDistsToPoint(x, y, lineObstacles)

        % In index form
        [r,c] = find(lineObstacles);
        
        % min distance from point from either side
        yDiff = r-y;
        xDiff = c-x;

        %% Doesn't work for straight vertical lines
        % isPositiveSlope = sum(yDiff>0) == sum(xDiff>0)
        % isNegativeSlope = sum(yDiff>0) == sum(xDiff<0)
        % if isPositiveSlope
        %     side1y = yDiff(yDiff>0 );
        %     side2y = yDiff(yDiff<=0);
        %     side1x = xDiff(xDiff>0 );
        %     side2x = xDiff(xDiff<=0);
        % elseif isNegativeSlope
        %     side1y = yDiff (yDiff>0 );
        %     side2y = yDiff (yDiff<=0);
        %     side1x = xDiff (xDiff<0 );
        %     side2x = xDiff (xDiff>=0);
        % else
        %     error('the number of x and y indicies to the left (or right) of the point (x,y) are not equal')
        % end % if
        
        % Will always have significant y differencews
        side1y = yDiff(yDiff>0 );
        side2y = yDiff(yDiff<=0);
        side1x = xDiff(yDiff>0 );
        side2x = xDiff(yDiff<=0);
        
        distsFromPointside1 = sqrt(side1y.^2 + side1x.^2);
        distsFromPointside2 = sqrt(side2y.^2 + side2x.^2);
        [side1Dist,side1Idx] = min(distsFromPointside1);
        [side2Dist,side2Idx] = min(distsFromPointside2);

        % The points - for debugging, currently.
        side1Row = side1y(side1Idx) + y;
        side1Col = side1x(side1Idx) + x;
        side2Row = side2y(side2Idx) + y;
        side2Col = side2x(side2Idx) + x;
end % function

% Give a point and an angle, return a binary image with a line drawn on it
function linemap = rasterlineFromPoint(y,x,angle,mapHeight,mapWidth)

    % % Method 1 (TODO does not work if line does not extend fully in height)
    % adjacent = mapHeight - y + 1;
    % opposite = adjacent * tand(angle);
    % p1 = [mapHeight, x + opposite - 1];

    % adjacent = y;
    % opposite = adjacent * tand(angle);
    % p2 = [1, x - opposite + 1];

    % [xx yy] = bresenham(p1(2), p1(1), p2(2), p2(1));

    % isInBounds = xx > 0 & yy > 0 & xx <= mapWidth & yy <= mapHeight;
    % xxx = xx(isInBounds);
    % yyy = yy(isInBounds);

    % linemap = zeros(mapHeight, mapWidth);
    % for j = 1:length(xxx)
    %     linemap(yyy(j),xxx(j)) = 1;
    % end

    % assert(angle <=90 && angle >=-90, 'angle must be between -90 and 90 degrees');


    % Hesse normal form to avoid infinite slope problems
    % angle is relative to positive y axis
    theta = angle - 90;
    r = y*sind(theta) - x*cosd(theta) ;

    getY = @(x)  ( r + x*cosd(theta) ) / sind(theta);
    getX = @(y) -( r - y*sind(theta) ) / cosd(theta);

    P1Y = getY(1);
    P1X = getX(1);
    P2Y = getY(mapHeight);
    P2X = getX(mapWidth);
    

    % x = slope * y + offset
    % y = (x - offset) / slope
    % slope = tand(angle);
    % assert(slope ~= Inf, 'infinite slope');
    % offset = x - slope * y;

    % getXr = @(y) slope * y + offset;
    % getYr = @(x) (x - offset) / slope;

    % P1Yr = getYr(1);
    % P1Xr = getXr(1);
    % P2Yr = getYr(mapHeight);
    % P2Xr = getXr(mapWidth);

    % isInXRange = P1X > 0 && P1X <= mapWidth;
    % isInYRange = P1Y > 0 && P1Y <= mapHeight;
    % if isInXRange
    %     p1 = [1, P1X];
    % elseif isInYRange
    %     p1 = [P1Y, 1];
    % else
    %     error('invalid point');
    % end

    isInXRange = P2X > 0 && P2X <= mapWidth;
    isInYRange = P2Y > 0 && P2Y <= mapHeight;

    % Get one extremum of the point
    mapHeightMinusY = mapHeight - y;
    mapWidthMinusX = mapWidth - x;
    yProjection1 = mapHeightMinusY;
    xProjection1 = yProjection1 * tand(angle);
    xProjection2 = mapWidthMinusX;
    yProjection2 = xProjection2 / tand(angle);
    isInXRange = (xProjection1 + x > 0) && xProjection1 <= mapWidthMinusX;
    isInYRange = (yProjection2 + y > 0) && yProjection2 <= mapHeightMinusY;
    if isInXRange
        p1 = [mapHeight, x + xProjection1 - 1];
    elseif isInYRange
        p1 = [y + yProjection2 - 1, mapWidth];
    else
        error('invalid point');
    end

    % Get the other extremum of the point
    adjacent1 = y;
    opposite1 = adjacent1 * tand(angle);
    opposite2 = x;
    adjacent2 = opposite2 / tand(angle);
    if opposite1 < x
        p2 = [1, x - opposite1 + 1];
    elseif adjacent2 < y
        p2 = [y - adjacent2 + 1, 1];
    else
        error('invalid point');
    end

    [xx yy] = bresenham(p1(2), p1(1), p2(2), p2(1));

    isInBounds = xx > 0 & yy > 0 & xx <= mapWidth & yy <= mapHeight;
    xxx = xx(isInBounds);
    yyy = yy(isInBounds);

    linemap = zeros(mapHeight, mapWidth);
    for j = 1:length(xxx)
        linemap(yyy(j),xxx(j)) = 1;
    end

end % function
