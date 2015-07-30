function [pointCloudRotated, newOrigin] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams)
% Downsamples and rotates the point cloud to algin the XY axes to the ground plane

    % ----------------------------------------
    % pointCloudVoxeled : downsampled
    % ----------------------------------------
    % apply voxel grid filter to a) speed up and b) place equal weights in each
    % voxel for RANSAC
    pointCloudObj = pointCloud(pointcloudRaw); % matlab pointcloud object
    pointCloudVoxeled = pcdownsample(pointCloudObj,'gridAverage',voxelGridSize);
    % pointCloudVoxeled = pointCloudObj; % don't downsample

    % ----------------------------------------
    % Rotate points to align ground plane to x-y plane
    % ----------------------------------------
    % Massage point cloud to be in proper format for getGroundPlane and rotatePointCloud
    pointCloudVoxeled = structuredToUnstructuredPointCloud(pointCloudVoxeled);
    pointCloudVoxeled = pointCloud(Rotate180AlongZ(pointCloudVoxeled.Location)); % To get positive Y values
    pointCloudVoxeledT = pointCloudVoxeled.Location'; % 3xN

    floorPlaneTolerance = ransacParams.floorPlaneTolerance; % tolerance in m
    maxInclinationAngle = ransacParams.maxInclinationAngle; % in degrees
    [updatedPlane, ~] = getGroundPlane(pointCloudVoxeledT, maxInclinationAngle, floorPlaneTolerance);

    % transform the point cloud to a more pratical reference system ---
    % Ground is now x-y plane.
    [pointsTRotated,R,Rinv] = rotatePointCloud(pointCloudVoxeledT, updatedPlane);
    oldOrigin = [0 0 0 1]';
    newOrigin = R*oldOrigin;

    pointCloudRotated = pointCloud(pointsTRotated');

    % ----------------------------------------
    % Plot
    % ----------------------------------------
    showPlots = true;
    if showPlots

        figure;
        subplot(1,3,1)
        titleString = 'Raw Point Cloud';
        plotPointCloud(pointcloudRaw, titleString);

        subplot(1,3,2)
        titleString = 'Voxelized Point Cloud with Ground Plane';
        plotPointCloud(pointCloudVoxeled, titleString);
        plotPlaneAroundPointCloud(updatedPlane, pointCloudVoxeled);

        subplot(1,3,3)
        titleString = 'Rotated relative to Ground';
        plotPointCloud(pointCloudRotated, titleString);

    end % if
end % function

% Needed to get positive Y values
% points: mx3 matrix of points
function [points] = Rotate180AlongZ(points)
    points(:,1) = points(:,1) * -1;
    points(:,2) = points(:,2) * -1;
end % function

% M x N x 3 to (N*M) x 3
function pointCloudDenoised = structuredToUnstructuredPointCloud(pointCloudObj)
    allIndicies = 1:pointCloudObj.Count;
    pointCloudDenoised = select(pointCloudObj, allIndicies);
end % function
