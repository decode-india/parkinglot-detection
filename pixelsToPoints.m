function [pointXs, pointYs] = pixelsToPoints(binaryMask, xScale, yScale)
    [maskRows, maskCols] = find(binaryMask);
    numPixels = sum(binaryMask(:));
    pointXs = zeros(numPixels,1);
    pointYs = zeros(numPixels,1);
    for i = 1:numPixels
        point = XYtoXYZ(maskRows(i), maskCols(i), xScale, yScale);
        pointXs(i) = point(1);
        pointYs(i) = point(2);
    end % for
end % function

function point = XYtoXYZ(yBox, xBox, xScale, yScale)
    pointX = xScale(xBox);
    pointY = yScale(yBox);
    point = [pointX, pointY];
end
