% Find the desirability for each configuration
function [measureAll] = findPotentialFunction(totalMap, origin, wheelchairShapeAngle)
    angle = 0; % for now
    posFromOrigin = 0;
    [numRows,numCols] = size(totalMap);
    numAngles = size(wheelchairShapeAngle, 3);
    
    [X, Y, THETA] = meshgrid(1:numCols,1:numRows, 1:numAngles);

    measureAll = zeros(numRows,numCols,numAngles);
    for r = 1:numRows
    for c = 1:numCols
    for theta = 1:numAngles
        measureAll(r,c,theta) =  computePotential(wheelchairShapeAngle, totalMap, c, r, theta);
    end % for
    end % for
    end % for

end % function

function measure = computePotential(wheelchairShapeAngle, totalMap, x, y, theta) 
    [numRows,numCols] = size(totalMap);
    
    wheelChair = zeros(numRows,numCols);
    wheelChair(y,x) = 1;
    wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,theta), 'same');
    wheelChair = wheelChair ~= 0;

    % figure
    % imshow(wheelChair);

    collisions = (totalMap(wheelChair) == 1);
    if any(collisions)
        measure = -Inf;
    else
        % measure = sumSquaredClosestDistance(wheelChair, totalMap);
        % [measure, distanceTransform] = sumSquaredClosestDistanceFlat(wheelChair, totalMap);
        [measure, distanceTransform] = minDistBetweenWheelchairAndObstacles(wheelChair, totalMap);

        % [measure, distanceTransform] = minSideDistBetweenWheelchairAndObstacles(wheelChair, totalMap);
        % [measure2, distanceTransform2] = minFrontDistBetweenWheelchairAndObstacles(wheelChair, totalMap);
        % measure = measure + measure2;

        % if (x == 70 & y == 50) | (x == 79 & y == 53)
        %     figure
        %     imshow(distanceTransform, [0 30], 'Colormap', parula);
        %     str = sprintf('Potential Function for Wheelchair (x,y): (%d, %d)', x, y);
        %     title(str);
        % end % if
    end
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
