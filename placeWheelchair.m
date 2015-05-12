function [possible] = placeWheelchair(wheelchairState, obstacleMap)
    [numRows, numCols, numAngles] = size(wheelchairState);

    numWheelchairsToFit = 1;
    [possible] = placeNwheelchairs(wheelchairState, obstacleMap, numWheelchairsToFit);
end % function

function [possible] = placeNwheelchairs(wheelchairState, obstaclesAndPreviousWheelchairs, N)
    if N == 0 
        possible = true;
        imshow(obstaclesAndPreviousWheelchairs);
        return
    end

    [numRows, numCols, numAngles] = size(wheelchairState);
    for r = 1:numRows
    for c = 1:numCols
    for theta = 1:numAngles
        wheelchair = logical(wheelchairState{r,c,theta});
        collisions = (obstaclesAndPreviousWheelchairs(wheelchair));
        if any(collisions)
            continue;
        end % if

        newObstacleMap = obstaclesAndPreviousWheelchairs | wheelchair;
        if placeNwheelchairs(wheelchairState, newObstacleMap, N-1);
            possible = true;
            return
        end % if

    end % for
    end % for
    end % for
    possible = false;
end % function
