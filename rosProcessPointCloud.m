
function rosProcessPointCloud(~, message, tftree) % callback
    pointcloudRaw = readXYZ(message);

    % voxelGridSize = 0.05; % in metres
    voxelGridSize = 0.05; % in metres
    ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
    ransacParams.maxInclinationAngle = 30; % in degrees
    [M, Minv, rot, T, no_best_plane] = processPointCloudLocal(pointcloudRaw, voxelGridSize, ransacParams);

    % if ~no_best_plane
        % Create a publisher, and publish out the rotated point cloud, as well as
        % the transform function. 
        quaternionCameraToGround = rotm2quat(Minv(1:3,1:3));
        translationCameraToGround = -T;
        % quaternionCameraToGround = [1 0 0 0];
        % translationCameraToGround = [0; 0; 2];

        % following http://www.mathworks.com/help/robotics/examples/accessing-the-tf-transformation-tree-in-ros.html
        tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg.ChildFrameId = 'ground_camera_link';
        tfStampedMsg.Header.FrameId = 'camera_depth_optical_frame';
        % tfStampedMsg.Header.FrameId = 'camera_depth_frame';
        % tfStampedMsg.Header.FrameId = 'camera_link';
        tfStampedMsg.Transform.Translation.X = translationCameraToGround(1);
        tfStampedMsg.Transform.Translation.Y = translationCameraToGround(2);
        tfStampedMsg.Transform.Translation.Z = translationCameraToGround(3);
        tfStampedMsg.Transform.Rotation.W = quaternionCameraToGround(1);
        tfStampedMsg.Transform.Rotation.X = quaternionCameraToGround(2);
        tfStampedMsg.Transform.Rotation.Y = quaternionCameraToGround(3);
        tfStampedMsg.Transform.Rotation.Z = quaternionCameraToGround(4);
        tfStampedMsg.Header.Stamp = rostime('now');

        % I can't seem to get tf publishing to work directly.
        sendTransform(tftree, tfStampedMsg);
        % send(groundTfPub,tfStampedMsg);
        % groundTfMsg = tfStampedMsg;
    % end % no_best_plane
end % function
