function [objectMap, groundMap, originMap] = pointCloudToObjectMap(pCloud, originPointcloud, metresPerMapUnit, groundThreshold)
    % ----------------------------------------
    % Easier names
    % ----------------------------------------
    % xScale: metres corresponding to each pixel
    xScale = pCloud.XLimits(1):metresPerMapUnit:pCloud.XLimits(2);
    yScale = pCloud.YLimits(1):metresPerMapUnit:pCloud.YLimits(2);
    zScale = pCloud.ZLimits(1):metresPerMapUnit:pCloud.ZLimits(2);
    xLength = length(xScale);
    yLength = length(yScale);

    % ----------------------------------------
    % objectMap and groundMap are 2D histograms. Put points in these
    % histograms.
    % ----------------------------------------
    objectMap = zeros(yLength, xLength);
    groundMap = zeros(yLength, xLength);
    points = pCloud.Location;
    for p = 1:pCloud.Count

        % find histogram bin
        [yBox, xBox] = XYZtoXY(points(p,:), xScale, yScale);

        % classify pixel
        isGroundPixel = abs(points(p,3)) < groundThreshold;
        if isGroundPixel
            groundMap(yBox,xBox) = groundMap(yBox,xBox) + 1; 
        % TODO if isAbove3metres, classify as roof
        else
            objectMap(yBox,xBox) = objectMap(yBox,xBox) + 1;
        end
    end % for

    % ----------------------------------------
    % Find histogram bin (location in map image) of specific point originPointcloud
    % TODO separate function
    % ----------------------------------------
    [yBox, xBox] = XYZtoXY(originPointcloud, xScale, yScale);
    originMap = [yBox, xBox]; % [y, x]
end % function

% Projects a point in 3D down onto a 2d map
% point: 3x1 (x,y,z)
% xScale: mx1 vector of bins in metres
% yScale: nx1 vector of bins in metres
% output: which bin to put the point 'point' in
function [yBox, xBox] = XYZtoXY(point, xScale, yScale)
    xBox = find(xScale >= point(1), 1, 'first');
    yBox = find(yScale >= point(2), 1, 'first');
end
