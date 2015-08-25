
cameraDepthRegisteredTopic = '/camera/depth_registered/points';
sub = rossubscriber(cameraDepthRegisteredTopic, rostype.sensor_msgs_PointCloud2);
message = receive(sub);

pointcloudRaw = readXYZ(message);
voxelGridSize = 0.05; % in metres
ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
ransacParams.maxInclinationAngle = 30; % in degrees
[pointCloudRotated, originPointcloud, R, Rinv] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams);

% mountToCamera = getTransform(tftree, 'camera_depth_optical_frame', 'camera_link')
rotationMatrix = R(1:3,1:3);
quaternionCameraToGround = rotm2quat(rotationMatrix);
translationCameraToGround = R(4,1:3);


% following http://www.mathworks.com/help/robotics/examples/accessing-the-tf-transformation-tree-in-ros.html
tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
tfStampedMsg.ChildFrameId = 'ground_camera_link';
tfStampedMsg.Header.FrameId = 'camera_link';
tfStampedMsg.Transform.Translation.X = translationCameraToGround(1);
tfStampedMsg.Transform.Translation.Y = translationCameraToGround(2);
tfStampedMsg.Transform.Translation.Z = translationCameraToGround(3);
tfStampedMsg.Transform.Rotation.W = quaternionCameraToGround(1);
tfStampedMsg.Transform.Rotation.X = quaternionCameraToGround(2);
tfStampedMsg.Transform.Rotation.Y = quaternionCameraToGround(3);
tfStampedMsg.Transform.Rotation.Z = quaternionCameraToGround(4);
tfStampedMsg.Header.Stamp = rostime('now');


tftree = rostf;
% tic;
% while toc < 20
% sendTransform(tftree, tfStampedMsg);
% end


% 'camera_depth_frame'
% 'camera_depth_optical_frame'
% 'camera_link'
% 'camera_rgb_frame'
% 'camera_rgb_optical_frame'
