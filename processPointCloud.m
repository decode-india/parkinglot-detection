function [pointCloudRotated, newOrigin] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams)
% Downsamples and rotates the point cloud to algin the XY axes to the ground plane

    % ----------------------------------------
    % pointCloudVoxeled : downsampled
    % ----------------------------------------
    % apply voxel grid filter to a) speed up and b) place equal weights in each
    % voxel for RANSAC
    pointCloudObj = pointCloud(pointcloudRaw); % matlab pointcloud object
    % voxelGridSize = 2; % m
    % pointCloudVoxeled = pcdownsample(pointCloudObj,'gridAverage',voxelGridSize);
    pointCloudVoxeled = pointCloudObj; % don't downsample

    pointCloudVoxeled = structuredToUnstructuredPointCloud(pointCloudVoxeled);

    % ----------------------------------------
    % pointCloudDenoised : removed noise
    % pointCloudDenoisedT : transpose of pointCloudDenoised
    % ----------------------------------------
    % Extra points occur near the origin of the camera - remove them.
    % radius = 0.10; % m
    % cameraPosition = [0 0 0];
    % originIndicies = findNeighborsInRadius(pointCloudVoxeled, cameraPosition, radius);
    % allIndicies = 1:pointCloudVoxeled.Count;
    % pointCloudDenoised = select(pointCloudVoxeled, setdiff(allIndicies,originIndicies));
    % pointCloudDenoisedT = pointCloudDenoised.Location'; % 3xN

    % ----------------------------------------
    % Rotate points to align ground plane to x-y plane
    % ----------------------------------------
    % Massage point cloud to be in proper format for getGroundPlane and rotatePointCloud
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
