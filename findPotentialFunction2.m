
function [qualityFunction] = findPotentialFunction2(wheelchairShapeAngleBig, objectMap, feasibleStates)
    [numRows,numCols] = size(objectMap);
    numAngles = size(wheelchairShapeAngle, 3);

    unknownMap = ~groundMap & ~objectMap;
    totalMap = ~groundMap;
    
    wheelchairDistTF = bwdist(wheelchairShapeAngleBig ~= 0, 'euclidean').^2; % distance to closest edge, square to bias large gaps
    objectDistTF = bwdist(objectMap, 'euclidean').^2; % distance to closest edge, square to bias large gaps

    measureAll = zeros(numRows,numCols,numAngles);
    for thetaIdx = 1:numAngles
        wheelChair(:,:,thetaIdx) = conv2(objectDistTF, wheelchairDistTF(:,:,thetaIdx), 'same');
    end % for
end % function
