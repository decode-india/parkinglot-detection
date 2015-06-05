% ASUS Xtion Pro has Minimum Depth Range. assume anything within 0.5m is viable.
% Input
%   point                : 1x2 double, location of origin in the image
%   [mapYSize, mapXSize] : size of image where the origin is located
%   radius               : radius of circle around origin
% Output
%   viablePixels         : mapYSize x mapXSize logical, 1's in a circle around
%                          origin
function [viablePixels] = circleAroundPoint(point, mapYSize, mapXSize, radius)
    [XX,YY] = meshgrid(1:mapXSize,1:mapYSize);
    distFromOrigin = sqrt((XX-point(2)).^2 + (YY-point(1)).^2);
    viablePixels = distFromOrigin < radius;
end % function
