% wheelchairMaps: numRows x numCols x numAngles cell array of (numRows x numCols) logical
function wheelchairMaps = getWheelChairMaps(wheelchairShapeAngle, numRows, numCols)
    numAngles = size(wheelchairShapeAngle, 3);
    wheelchairMaps = cell(numRows, numCols, numAngles);

    % TODO this conv2 is the bottleneck. speed up.
    for thetaIdx = 1:numAngles
        thetaIdx
    for r = 1:numRows
    for c = 1:numCols
        wheelChair = zeros(numRows,numCols);
        wheelChair(r,c) = 1;
        wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,thetaIdx), 'same');
        wheelChair = wheelChair ~= 0;
        wheelchairMaps{r,c,thetaIdx} = wheelChair;
    end % for
    end % for
    end % for
end % function
