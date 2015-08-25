function [pointCloudRotated, newOrigin, R_final, T_final] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams)
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
    pointCloudVoxeledT = structuredToUnstructuredPointCloud(pointCloudVoxeled);
    pointCloudVoxeledT = pointCloudVoxeledT.Location'; % 3xN, for easy application of rotations

    R_aboutZ = [-1 0 0; 0 -1 0; 0 0 1];
    M_aboutZ = [R_aboutZ zeros(3,1); 0 0 0 1];
    pointCloudVoxeledT2 = R_aboutZ * pointCloudVoxeledT;

    floorPlaneTolerance = ransacParams.floorPlaneTolerance; % tolerance in m
    maxInclinationAngle = ransacParams.maxInclinationAngle; % in degrees
    [updatedPlane] = getGroundPlane(pointCloudVoxeledT2, maxInclinationAngle, floorPlaneTolerance);

    % transform the point cloud to a more pratical reference system ---
    % Ground is now x-y plane.
    % [pointsTRotated,R,Rinv] = rotatePointCloud(pointCloudVoxeledT, updatedPlane);
    [M, Minv, rot, T] = getRotationMatrix(updatedPlane);
    M_final = M * M_aboutZ;
    Minv_final = Minv * M_aboutZ;
    R_final = rot * R_aboutZ;
    T_final = T;
    pointsTRotated = R_final*pointCloudVoxeledT + repmat(T_final,1,size(pointCloudVoxeledT,2));
    oldOrigin = [0 0 0 1]';
    newOrigin = M*oldOrigin;

    pointCloudRotated = pointCloud(pointsTRotated');

    % ----------------------------------------
    % Plot
    % ----------------------------------------
    showPlots = false;
    if showPlots
 
        figure(3);
        subplot(1,3,1)
        titleString = 'Raw Point Cloud';
        plotPointCloud(pointcloudRaw, titleString);

        subplot(1,3,2)
        titleString = 'Voxelized Point Cloud with Ground Plane';
        pointCloudSemiRotated = pointCloud(pointCloudVoxeledT2');
        plotPointCloud(pointCloudSemiRotated, titleString);
        plotPlaneAroundPointCloud(updatedPlane, pointCloudSemiRotated);

        subplot(1,3,3)
        titleString = 'Rotated relative to Ground';
        plotPointCloud(pointCloudRotated, titleString);

    end % if
end % function


