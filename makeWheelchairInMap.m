% UNUSED
% places wheelchair in a (numRows x numCols) 2D array of zeros at location (r,c).
% wheelchairshape: nxm double
% r, c: integers
% numRows, numCols: integers
% output: bool
function wheelchairInMap = getWheelChairMaps(wheelchairshape, r, c, numRows, numCols)
    wheelChair = zeros(numRows,numCols);
    wheelChair(r,c) = 1;
    wheelChair = conv2(wheelChair, wheelchairshape, 'same');
    wheelchairInMap = wheelChair ~= 0;
end % function
