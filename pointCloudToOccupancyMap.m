function [occupancyMap, groundMap, origin] = pointCloudToOccupancyMap(pCloud, gridStep, newOrigin)
    xScale = pCloud.XLimits(1):gridStep:pCloud.XLimits(2);
    yScale = pCloud.YLimits(1):gridStep:pCloud.YLimits(2);
    zScale = pCloud.ZLimits(1):gridStep:pCloud.ZLimits(2);

    xLength = length(xScale);
    yLength = length(yScale);
    occupancyMap = zeros(yLength, xLength);
    groundMap = zeros(yLength, xLength);
    for p = 1:pCloud.Count
        point = pCloud.Location(p,:);
        xBox = find(xScale >= point(1), 1, 'first');
        yBox = find(yScale >= point(2), 1, 'first');
        isGroundPixel = abs(point(3)) < 10;
        if isGroundPixel
            groundMap(yBox,xBox) = groundMap(yBox,xBox) + 1; 
        else
            occupancyMap(yBox,xBox) = occupancyMap(yBox,xBox) + 1;
        end
    end % for

    xBox = find(xScale >= newOrigin(1), 1, 'first');
    yBox = find(yScale >= newOrigin(2), 1, 'first');
    origin = [yBox, xBox]; % [y, x]
end % function
