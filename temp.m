
function isFeasibleState = temp(r, c, thetaIdx, wheelchairShapeAngle, numRows, numCols, groundMap)
    impulseMap = zeros(numRows,numCols);
    impulseMap(r,c) = 1;

    placedWheelchair = conv2(impulseMap, wheelchairShapeAngle(:,:,thetaIdx),'same');

    negGroundMap = ~groundMap; % free space == 1
    collisions = negGroundMap(placedWheelchair ~= 0) == 1;

    % if (groundMap(r,c))
    % % if any(collisions)
    %     isFeasibleState = false;
    % else
    %     isFeasibleState = true;
    % end 
    isFeasibleState = r + c;
end
