

waitForTransform(tftree, 'ground_camera_link', 'camera_link');

pcMessage = receive(processPointCloudSub, 10);

% pointcloudRaw = readXYZ(message);
% voxelGridSize = 0.01; % in metres
% ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
% ransacParams.maxInclinationAngle = 30; % in degrees
% 
% pointCloudObj = pointCloud(pointcloudRaw); % matlab pointcloud object
% pointCloudVoxeled = pcdownsample(pointCloudObj,'gridAverage',voxelGridSize);
% % pointCloudVoxeled = pointCloudObj; % don't downsample

tfpt = transform(tftree, 'ground_camera_link', pcMessage);

% % ----------------------------------------
% % Rotate points to align ground plane to x-y plane
% % ----------------------------------------
% % Massage point cloud to be in proper format for getGroundPlane and rotatePointCloud
% pointCloudVoxeled = structuredToUnstructuredPointCloud(pointCloudVoxeled);
% pointCloudVoxeled = pointCloud(rotate180AlongZ(pointCloudVoxeled.Location)); % To get positive Y values
% pointCloudVoxeledT = pointCloudVoxeled.Location'; % 3xN
% 
% pointsTRotated = rot*pointCloudVoxeledT + repmat(T,1,size(pointCloudVoxeledT,2));
% oldOrigin = [0 0 0 1]';
% newOrigin = M*oldOrigin;
% 
% pointCloudRotated = pointCloud(pointsTRotated');

pointCloudRotated = readXYZ(pcMessage);
pointcloudRaw = readXYZ(tfpt);

% ----------------------------------------
% Plot
% ----------------------------------------
showPlots = true;
if showPlots

    figure(1);
    subplot(1,2,1)
    titleString = 'Raw Point Cloud';
    plotPointCloud(pointcloudRaw, titleString);

    % subplot(1,3,2)
    % titleString = 'Voxelized Point Cloud with Ground Plane';
    % plotPointCloud(pointCloudVoxeled, titleString);
    % plotPlaneAroundPointCloud(updatedPlane, pointCloudVoxeled);

    subplot(1,2,2)
    titleString = 'Rotated relative to Ground';
    plotPointCloud(pointCloudRotated, titleString);

end % if
