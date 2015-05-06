function [occupancyMap, groundMap, origin] = depthToOccupancyMap(imDepth)

    % ----------------------------------------
    % Flip
    % going from (0,0) in top left to (0,0) in bottom right
    % ----------------------------------------
    isUpsideDown = true; 
    isMirrored = true;
    imDepthFlipped = flipImage(imDepth, isUpsideDown, isMirrored);

    % ----------------------------------------
    % Convert to Point Cloud
    % ----------------------------------------
    [pointcloudRaw, ~] = depthToCloud(imDepthFlipped);

    % ----------------------------------------
    % pointCloudVoxeled : downsampled
    % ----------------------------------------
    pointCloudObj = pointCloud(pointcloudRaw); % matlab pointcloud object
    gridStep = 2;
    pointCloudVoxeled = pcdownsample(pointCloudObj,'gridAverage',gridStep);
    % pointCloudVoxeled = pointCloudObj; % don't downsample

    % ----------------------------------------
    % pointCloudDenoised : removed noise
    % pointCloudDenoisedT : transpose of pointCloudDenoised
    % ----------------------------------------
    % Extra points occur near the origin of the camera - remove them.
    radius = 10; % cm
    cameraPosition = [0 0 0];
    originIndicies = findNeighborsInRadius(pointCloudVoxeled, cameraPosition, radius);
    allIndicies = 1:pointCloudVoxeled.Count;
    pointCloudDenoised = select(pointCloudVoxeled, setdiff(allIndicies,originIndicies));
    pointCloudDenoisedT = pointCloudDenoised.Location'; % 3xN

    % ----------------------------------------
    % Rotate points to align ground plane to x-y plane
    % ----------------------------------------
    floorPlaneTolerance = 2; % tolerance in mm
    maxInclinationAngle = 40; % in degrees
    [updatedPlane, ~] = getGroundPlane(pointCloudDenoisedT, maxInclinationAngle, floorPlaneTolerance);

    % transform the point cloud to a more pratical reference system ---
    % Ground is now x-y plane.
    [pointsTRotated,R,Rinv] = rotatePointCloud(pointCloudDenoisedT, updatedPlane);
    oldOrigin = [0 0 0 1]';
    newOrigin = R*oldOrigin;

    pointCloudRotated = pointCloud(pointsTRotated');
    gridStepMap = 2;
    % groundMap: the visible ground
    % occupancyMap: what's occupied above ground
    [occupancyMap, groundMap, origin] = pointCloudToOccupancyMap(pointCloudRotated, gridStepMap, newOrigin);

end % function
