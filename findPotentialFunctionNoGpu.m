function [measureAll] = findPotentialFunctionNoGpu(objectMap, wheelchairShapeAngleBig, feasibleStates)
    [numRows,numCols] = size(objectMap);
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
            mapAndChair = wheelChair | objectMap;
            distanceTransform = bwdist(mapAndChair).^2; % distance to closest edge, square to bias large gaps
            measureAll(r,c,thetaIdx) = sum(distanceTransform(:)) / nnz(distanceTransform);
        end % if
    end % for
    end % for
    end % for

end % function
function [measure, distanceTransform] = sumSquaredClosestDistance2(wheelChair, objectMap)
    mapAndChair = wheelChair | objectMap;
    % figure
    % imshow(mapAndChair);

    distanceTransform = bwdist(mapAndChair).^2; % distance to closest edge, square to bias large gaps
    % figure
    % imshow(distanceTransform, [], 'Colormap', jet(255));

    % normalize by number of pixels TODO
    measure = sum(distanceTransform(:)) / nnz(distanceTransform);
end % function
