% wheelChairMaps: a numRows x numCols map for each state
% groundMap: numRows x numCols, a map of ground pixels. 1 == ground, 0 == not
% origin: 1x2 double, [y x] position of camera in map
function [feasibleStates2] = findFeasibleStates(groundMap, wheelChairMaps, origin)
    [numRows, numCols, numAngles] = size(wheelChairMaps);

    % TODO RRT if needed
    
    % If any wheelChairMap isn't fully on the ground, it's infeasible.
    feasibleStates = zeros(numRows, numCols, numAngles);
    for thetaIdx = 1:numAngles
    for r = 1:numRows
    for c = 1:numCols
        feasible = true;
        wheelChairMap = wheelChairMaps{r, c, thetaIdx};
        negGroundMap = ~groundMap; % free space == 1
        collisions = negGroundMap(wheelChairMap) == 1;
        if any(collisions)
            feasible = false;
        end 
        feasibleStates(r, c, thetaIdx) = feasible;
    end % for
    end % for
    end % for

    % The feasible states are the ones connected to the origin in the
    % configuration space. Flood-fill the feasible states starting from the
    % origin. 
    originAngle = ceil(numAngles/2);
    origState = [origin(1), origin(2), originAngle];
    assert(feasibleStates(origState(1), origState(2), origState(3)) == 1, 'Origin state collides with wall');

    connectivityNeighbourhood = 6; % 6-connected neighbourhood (1 manhattan distance in 3d)
    filledRegion = imfill(~feasibleStates, origState, connectivityNeighbourhood); % ~feasibleStates | region connected to origin 
    feasibleStates2 = feasibleStates & filledRegion; % region connected to origin

end % function
