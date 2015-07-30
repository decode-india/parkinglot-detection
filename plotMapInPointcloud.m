function plotMapInPointcloud(binaryMask, pCloud, metresPerMapUnit, colour, depths)
    
    % TODO remove dependency on pCloud; we only need the limits
    xScale = pCloud.XLimits(1):metresPerMapUnit:pCloud.XLimits(2);
    yScale = pCloud.YLimits(1):metresPerMapUnit:pCloud.YLimits(2);
    zScale = pCloud.ZLimits(1):metresPerMapUnit:pCloud.ZLimits(2);
    xLength = length(xScale);
    yLength = length(yScale);

    [mapYSize, mapXSize] = size(binaryMask);
    assert(xLength == mapXSize, 'map x sizes not equal');
    assert(yLength == mapYSize, 'map y sizes not equal');

    % Get X-Y points
    [pointXs, pointYs] = pixelsToPoints(binaryMask, xScale, yScale);

    % Z points are constant.
    numPixels = sum(binaryMask(:));
    pointZs = zeros(numPixels,1);

    hold on
    % for i = 0.05:metresPerMapUnit:0.1
    for i = depths
        scatter3(pointXs, pointYs, pointZs + i, 'filled', 'MarkerFaceColor', colour);
    end % for
end
