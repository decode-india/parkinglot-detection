
% ipaddress = 'http://localhost:11311';
% rosinit(ipaddress);

% cameraDepthRegisteredTopic = '/camera/depth_registered/points';
cameraDepthRegisteredTopic = '/camera/depth/points';

% groundTfPub = rospublisher('/ground_tf',rostype.geometry_msgs_TransformStamped);
% processPointCloudSub = rossubscriber(cameraDepthRegisteredTopic, {@rosProcessPointCloud, groundTfPub});
% chatterpub = rospublisher('/chatter',rostype.std_msgs_String)

tftree = rostf;
processPointCloudSub = rossubscriber(cameraDepthRegisteredTopic, {@rosProcessPointCloud, tftree});

% % while forever, loop and push out ground_tf to tf.
% tftree = rostf;
% global quaternionCameraToGround;
% global translationCameraToGround;
% tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
% tfStampedMsg.ChildFrameId = 'ground_camera_link';
% tfStampedMsg.Header.FrameId = 'camera_link';
% tic;
% while toc < 40
%     tfStampedMsg.Transform.Translation.X = translationCameraToGround(1);
%     tfStampedMsg.Transform.Translation.Y = translationCameraToGround(2);
%     tfStampedMsg.Transform.Translation.Z = translationCameraToGround(3);
%     tfStampedMsg.Transform.Rotation.W = quaternionCameraToGround(1);
%     tfStampedMsg.Transform.Rotation.X = quaternionCameraToGround(2);
%     tfStampedMsg.Transform.Rotation.Y = quaternionCameraToGround(3);
%     tfStampedMsg.Transform.Rotation.Z = quaternionCameraToGround(4);
%     tfStampedMsg.Header.Stamp = rostime('now');
%     sendTransform(tftree, tfStampedMsg);
% end
