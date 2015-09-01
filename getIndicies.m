% Object map Distance Transform : numRows x numCols
% Wheelchair Distance Transform : 2*(numRows-1)+1 x 2*(numCols-1)+1
% for a given row/column in Object Map DT, generate indicies in WC DT
function [lowerIndex, upperIndex] = getIndicies(r, numRows)
    paddingPerSide = (numRows-1);
    wheelchairRows = 2*paddingPerSide+1;

    lowerIndex = numRows - r + 1;
    upperIndex = wheelchairRows - r + 1;
end % function
