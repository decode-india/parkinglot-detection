% Find the desirability for each configuration
function [measureAll] = findPotentialFunction(totalMap, origin, wheelchairShape)
    angle = 0; % for now
    posFromOrigin = 0;
    [ySize,xSize] = size(totalMap);
    
    % measure = computePotential(wheelchairShape, totalMap, 154, 84) 
    % measure = computePotential(wheelchairShape, totalMap, 5, 20) 

    [X,Y] = meshgrid(1:xSize,1:ySize);
    % X = gpuArray(X);
    % Y = gpuArray(Y);

    potentialAt = @(x,y) computePotential(wheelchairShape, totalMap, x, y);
    measureAll = arrayfun(potentialAt, X, Y);

end % function

function measure = computePotential(wheelchairShape, totalMap, x, y) 
    [ySize,xSize] = size(totalMap);
    
    % wheelChair is wheelchairShape at position (y,x), zeros elsewhere
    wheelChair = zeros(ySize,xSize);
    wheelChair(y,x) = 1;
    wheelChair = conv2(wheelChair, wheelchairShape, 'same');
    wheelChair = wheelChair ~= 0;

    % figure
    % imshow(wheelChair);

    collisions = (totalMap(wheelChair) == 1);
    if any(collisions)
        measure = 0;
    else
        % measure = sumSquaredClosestDistance(wheelChair, totalMap);
        % measure = sumSquaredClosestDistanceFlat(wheelChair, totalMap);
        measure = minDistBetweenWheelchairAndObstacles(wheelChair, totalMap);
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
function measure = sumSquaredClosestDistanceFlat(wheelChair, totalMap)
    mapAndChair = wheelChair | totalMap;
    mapAndChair = gpuArray(mapAndChair);

    distanceTransform = bwdist(mapAndChair).^2; % distance to closest edge, square to bias large gaps
    measure = sum(distanceTransform(:));
    measure = double(gather(measure));

    threshold = 5; % pixels to wall
    isLessThanThreshold = distanceTransform > 0 & distanceTransform <= threshold.^2;
    constValue = 1;
    distanceTransform(isLessThanThreshold) = constValue; % _/
    % distanceTransform(isLessThanThreshold) = threshold^2 - % distanceTransform(isLessThanThreshold); % \_/ 
end

function measure = minDistBetweenWheelchairAndObstacles(wheelChair, totalMap)
    % figure
    % imshow(wheelChair);
    % figure
    % imshow(totalMap);

    % get wheelchair perimeter
    wheelChairPerimeter = bwperim(wheelChair,8);
    wheelChairPerimeter = gpuArray(wheelChairPerimeter);
    totalMap = gpuArray(totalMap);
    distanceTransform = bwdist(totalMap); % distance to closest obstacle
    distanceTransform = distanceTransform .* wheelChairPerimeter;
    % figure
    % imshow(distanceTransform, [], 'Colormap', jet(255));

    % measure = sum(distanceTransform(:));
    measure = min(distanceTransform(wheelChairPerimeter));
    measure = double(gather(measure));
end
