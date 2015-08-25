% Needed to get positive Y values
% points: mx3 matrix of points
function [points] = rotate180AlongZ(points)
    points(:,1) = points(:,1) * -1;
    points(:,2) = points(:,2) * -1;
end % function
