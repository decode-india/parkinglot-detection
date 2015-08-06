% wheelChairMaps: a numRows x numCols map for each state
% groundMap: numRows x numCols, a map of ground pixels. 1 == ground, 0 == not
% origin: 1x2 double, [y x] position of camera in map
function [feasibleStates2] = findFeasibleStates(groundMap, wheelchairShapeAngle, origin)
    [numRows, numCols] = size(groundMap);
    numAngles = size(wheelchairShapeAngle,3);

    % % TODO RRT if needed

    negGroundMap = ~groundMap; % free space == 0
    feasibleStates = false(numRows, numCols, numAngles);
    for thetaIdx = 1:numAngles
        feasibleStatesAngle = conv2(double(negGroundMap), wheelchairShapeAngle(:,:,thetaIdx), 'same');
        feasibleStates(:, :, thetaIdx) = ~logical(feasibleStatesAngle);
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

    % Not feasible if wheelchair is not fully on map
    % If wheelchair collides with borders, state is not feasible
    nonBorderStates = false(numRows, numCols);
    for thetaIdx = 1:numAngles
        boundaryValue = 1;
        boundaryStates(:,:,thetaIdx) = imfilter(nonBorderStates, wheelchairShapeAngle(:,:,thetaIdx), boundaryValue);
    end % for

    feasibleStates2(boundaryStates) = false;
end % function
