function [M_final, Minv_final, R_final, T_final, no_best_plane] = processPointCloudLocal(pointcloudRaw, voxelGridSize, ransacParams)
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
    % pointCloudVoxeled = pointCloud(rotate180AlongZ(pointCloudVoxeled.Location)); % To get positive Y values

    pointCloudVoxeledT = structuredToUnstructuredPointCloud(pointCloudVoxeled);
    pointCloudVoxeledT = pointCloudVoxeledT.Location'; % 3xN, for easy application of rotations

    R_aboutZ = [-1 0 0; 0 -1 0; 0 0 1];
    M_aboutZ = [R_aboutZ zeros(3,1); 0 0 0 1];
    pointCloudVoxeledT2 = R_aboutZ * pointCloudVoxeledT;

    floorPlaneTolerance = ransacParams.floorPlaneTolerance; % tolerance in m
    maxInclinationAngle = ransacParams.maxInclinationAngle; % in degrees
    [updatedPlane, ~, no_best_plane] = getGroundPlane(pointCloudVoxeledT2, maxInclinationAngle, floorPlaneTolerance);

    % transform the point cloud to a more pratical reference system ---
    % Ground is now x-y plane.
    % pointCloudRotated = rot * R_aboutZ * pointCloudVoxeledT
    if ~no_best_plane
        [M, Minv, rot, T] = getRotationMatrix(updatedPlane);
        M_final = M * M_aboutZ;
        Minv_final = Minv * M_aboutZ;
        R_final = rot * R_aboutZ;
        T_final = T;
    else
        M = eye(4);
        Minv = M;
        rot = eye(3);
        T = [0;0;0];
    end
end
