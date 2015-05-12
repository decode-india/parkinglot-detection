% TODO optimize, takes long
% wheelchairShapeAngle: n x m x numAngles, in material space
% wheelchairState: {numRows x numCols x numAngles}(numRows,numCols), in configuration space
% output is of type double; aliased at sides
function wheelchairState = wheelchairShapeToState(wheelchairShapeAngle, numRows, numCols)
    numAngles = size(wheelchairShapeAngle,3);
    wheelchairState = cell(numRows,numCols,numAngles);
    for r = 1:numRows
    for c = 1:numCols
    for theta = 1:numAngles
        wheelChair = zeros(numRows,numCols);
        wheelChair(r,c) = 1;
        wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,theta), 'same');
        wheelchairState{r,c,theta} = wheelChair;
    end
    end
    end % for
end % function
