function [pointCloudRotated, newOrigin] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams)
% Downsamples and rotates the point cloud to algin the XY axes to the ground plane

    % ----------------------------------------
    % pointCloudVoxeled : downsampled
    % ----------------------------------------
    % apply voxel grid filter to a) speed up and b) place equal weights in each
    % voxel for RANSAC
    pointCloudObj = pointCloud(pointcloudRaw); % matlab pointcloud object
    % voxelGridSize = 2; % m
    pointCloudVoxeled = pcdownsample(pointCloudObj,'gridAverage',voxelGridSize);
    pointCloudVoxeled = pointCloudObj; % don't downsample

    % pointCloudVoxeledRow = select(pointCloudVoxeled, 1:pointCloudVoxeled.Count); % hxwx3 to mx3
    % pointCloudVoxeledT(:,2) = -pointCloudVoxeledT(:,2);  % flip so Y is positive in upwards direction
    % pointCloudDenoisedT = pointCloudVoxeledT;

    % ----------------------------------------
    % pointCloudDenoised : removed noise
    % pointCloudDenoisedT : transpose of pointCloudDenoised
    % ----------------------------------------
    % Extra points occur near the origin of the camera - remove them.
    radius = 0.10; % m
    cameraPosition = [0 0 0];
    originIndicies = findNeighborsInRadius(pointCloudVoxeled, cameraPosition, radius);
    allIndicies = 1:pointCloudVoxeled.Count;
    pointCloudDenoised = select(pointCloudVoxeled, setdiff(allIndicies,originIndicies));
    pointCloudDenoisedT = pointCloudDenoised.Location'; % 3xN

    % ----------------------------------------
    % Rotate points to align ground plane to x-y plane
    % ----------------------------------------
    % floorPlaneTolerance = 0.02; % tolerance in m
    floorPlaneTolerance = ransacParams.floorPlaneTolerance; % tolerance in m
    maxInclinationAngle = ransacParams.maxInclinationAngle; % in degrees
    [updatedPlane, ~] = getGroundPlane(pointCloudDenoisedT, maxInclinationAngle, floorPlaneTolerance);

    % transform the point cloud to a more pratical reference system ---
    % Ground is now x-y plane.
    [pointsTRotated,R,Rinv] = rotatePointCloud(pointCloudDenoisedT, updatedPlane);
    oldOrigin = [0 0 0 1]';
    newOrigin = R*oldOrigin;

    pointCloudRotated = pointCloud(pointsTRotated');


    % ----------------------------------------
    % Plot
    % ----------------------------------------
    showPlots = true;
    if showPlots
        figure;
        subplot(2,1,1)
        showPointCloud(pointCloudVoxeled);
        colormap(parula)
        title('Voxelized Point Cloud with Ground Plane');
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        axis equal;

        % Plot 
        p = pointCloudVoxeled;
        a = updatedPlane(1);
        b = updatedPlane(2);
        c = updatedPlane(3);
        d = updatedPlane(4);
        numTicks = 100;
        xScale = linspace(p.XLimits(1),p.XLimits(2), numTicks);
        yScale = linspace(p.YLimits(1),p.YLimits(2), numTicks);
        zScale = linspace(p.ZLimits(1),p.ZLimits(2), numTicks);
        [xx,yy,zz] = meshgrid(xScale, yScale, zScale);
        isosurface(xx, yy, zz, a*xx+b*yy+c*zz+d, 0)

        % Plot Point Cluod
        subplot(2,1,2)
        showPointCloud(pointCloudRotated);
        colormap(parula)
        title('Rotated relative to Ground');
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        axis equal;
    end % if
end % function
