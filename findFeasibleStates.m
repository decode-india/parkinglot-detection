% wheelChairMaps: a numRows x numCols map for each state
% groundMap: numRows x numCols, a map of ground pixels. 1 == ground, 0 == not
% origin: 1x2 double, [y x] position of camera in map
function [feasibleStates2] = findFeasibleStates(groundMap, wheelchairShapeAngle, origin)
    [numRows, numCols] = size(groundMap);
    numAngles = size(wheelchairShapeAngle,3);

    wheelChairMaps = getWheelChairMaps(wheelchairShapeAngle, numRows, numCols);

    % % TODO RRT if needed

    % feasibleStates = false(numRows, numCols, numAngles);
    % for thetaIdx = 1:numAngles
    % for r = 1:numRows
    % for c = 1:numCols
    %     feasible = true;
    %     wheelChairMap = wheelChairMaps{r, c, thetaIdx};

    %     % If any wheelChairMap isn't fully on the ground, it's infeasible.
    %     negGroundMap = ~groundMap; % free space == 0
    %     collisions = negGroundMap(wheelChairMap) == 1;
    %     if any(collisions)
    %         feasible = false;
    %     end 

    %     feasibleStates(r, c, thetaIdx) = feasible;
    % end % for
    % end % for
    % end % for


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
    feasibleStates3 = false(numRows, numCols, numAngles);
    for thetaIdx = 1:numAngles
        centreWheelChairMap = wheelChairMaps{floor(numRows/2), floor(numCols/2), thetaIdx};
        wheelChairArea = sum(centreWheelChairMap(:));
        for r = 1:numRows
        for c = 1:numCols
            wheelChairMap = wheelChairMaps{r, c, thetaIdx};

            % If convolution is cut off, it's infeasible
            if sum(wheelChairMap(:)) < wheelChairArea
                feasibleStates3(r, c, thetaIdx) = false;
            elseif sum(wheelChairMap(:)) > wheelChairArea
                error('bad wheelchair size');
            else
                feasibleStates3(r, c, thetaIdx) = true;
            end % if
        end % for
        end % for
    end % for

    feasibleStates2(~feasibleStates3) = false;
end % function

function isFeasibleState = overlapsWheelchair(r, c, thetaIdx, wheelchairShapeAngle, numRows, numCols, groundMap)
    % impulseMap = zeros(numRows,numCols);
    % impulseMap(r,c) = 1;

    % placedWheelchair = conv2(impulseMap, wheelchairShapeAngle(:,:,thetaIdx),'same');

    % negGroundMap = ~groundMap; % free space == 1
    % collisions = negGroundMap(placedWheelchair ~= 0) == 1;

    % if (groundMap(r,c))
    % % if any(collisions)
    %     isFeasibleState = false;
    % else
    %     isFeasibleState = true;
    % end 
    isFeasibleState = groundMap(r,c);
end % function
