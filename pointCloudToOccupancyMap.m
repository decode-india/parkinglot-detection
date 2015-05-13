function [occupancyMap, groundMap, origin] = pointCloudToOccupancyMap(pCloud, newOrigin, gridStep, groundThreshold)
    % ----------------------------------------
    % Easier names
    % ----------------------------------------
    xScale = pCloud.XLimits(1):gridStep:pCloud.XLimits(2);
    yScale = pCloud.YLimits(1):gridStep:pCloud.YLimits(2);
    zScale = pCloud.ZLimits(1):gridStep:pCloud.ZLimits(2);
    xLength = length(xScale);
    yLength = length(yScale);

    % ----------------------------------------
    % occupancyMap and groundMap are 2D histograms. Put points in these
    % histograms.
    % ----------------------------------------
    occupancyMap = zeros(yLength, xLength);
    groundMap = zeros(yLength, xLength);
    for p = 1:pCloud.Count
        point = pCloud.Location(p,:);
        xBox = find(xScale >= point(1), 1, 'first');
        yBox = find(yScale >= point(2), 1, 'first');
        isGroundPixel = abs(point(3)) < groundThreshold;
        if isGroundPixel
            groundMap(yBox,xBox) = groundMap(yBox,xBox) + 1; 
        else
            occupancyMap(yBox,xBox) = occupancyMap(yBox,xBox) + 1;
        end
    end % for

    % ----------------------------------------
    % Find histogram bin (location in map image) of specific point newOrigin
    % TODO separate function
    % ----------------------------------------
    xBox = find(xScale >= newOrigin(1), 1, 'first');
    yBox = find(yScale >= newOrigin(2), 1, 'first');
    origin = [yBox, xBox]; % [y, x]

    % ----------------------------------------
    % Plot
    % ----------------------------------------
    showPlots = true;
    if showPlots
        figure;
        subplot(2,1,1)
        imshow(occupancyMap)
        title('Occupancy Map');
        subplot(2,1,2)
        imshow(groundMap)
        title('Ground Map');
    end
end % function
