function [objectMap, groundMap, origin] = pointCloudToObjectMap(pCloud, newOrigin, gridStep, groundThreshold)
    % ----------------------------------------
    % Easier names
    % ----------------------------------------
    % xScale: metres corresponding to each pixel
    xScale = pCloud.XLimits(1):gridStep:pCloud.XLimits(2);
    yScale = pCloud.YLimits(1):gridStep:pCloud.YLimits(2);
    zScale = pCloud.ZLimits(1):gridStep:pCloud.ZLimits(2);
    xLength = length(xScale);
    yLength = length(yScale);

    % ----------------------------------------
    % objectMap and groundMap are 2D histograms. Put points in these
    % histograms.
    % ----------------------------------------
    objectMap = zeros(yLength, xLength);
    groundMap = zeros(yLength, xLength);
    for p = 1:pCloud.Count
        point = pCloud.Location(p,:);

        % find histogram bin
        [yBox, xBox] = XYZtoXY(point, xScale, yScale);

        % classify pixel
        isGroundPixel = abs(point(3)) < groundThreshold;
        if isGroundPixel
            groundMap(yBox,xBox) = groundMap(yBox,xBox) + 1; 
        % TODO if isAbove3metres, classify as roof
        else
            objectMap(yBox,xBox) = objectMap(yBox,xBox) + 1;
        end
    end % for

    % ----------------------------------------
    % Find histogram bin (location in map image) of specific point newOrigin
    % TODO separate function
    % ----------------------------------------
    xBox = find(xScale >= newOrigin(1), 1, 'first');
    yBox = find(yScale >= newOrigin(2), 1, 'first');
    origin = [yBox, xBox]; % [y, x]
end % function

function [yBox, xBox] = XYZtoXY(point, xScale, yScale)
    xBox = find(xScale >= point(1), 1, 'first');
    yBox = find(yScale >= point(2), 1, 'first');
end
