close all;
outputFolder = '/home/vgan/code/experiments/parkinglot-detection-output';
dateFolder = '20150826';
saveFolder = fullfile(outputFolder, dateFolder, 'for-realtime');
makeFolder(saveFolder);

timeOut = 3; % seconds
pointcloudTopic = '/camera/depth/points';
pointcloudSub = rossubscriber(pointcloudTopic);
pointCloudMsgOptic = receive(pointcloudSub,timeOut);
clear('pointcloudSub')

tftree = rostf;
waitForTransform(tftree, 'odom', 'camera_depth_optical_frame');
pointcloudMsgOdom = transform(tftree, 'odom', pointCloudMsgOptic);

% Get OtoO : transform from odom to camera_depth_optical_frame
tf_OdomToOptic = getTransform(tftree,'camera_depth_optical_frame', 'odom');
trans = tf_OdomToOptic.Transform.Translation;
T_OdomToOptic= [trans.X; trans.Y; trans.Z];
quat = tf_OdomToOptic.Transform.Rotation;
OtoOquat = [quat.W quat.X quat.Y quat.Z];
R_OdomToOptic = quat2rotm(OtoOquat);
clear('tftree')

pointCloudMsgOptic.PreserveStructureOnRead = true;
xyzOptic = readXYZ(pointCloudMsgOptic);

[chosenStateWorldX, chosenStateWorldY] = findParkingSpot(xyzOptic, saveFolder);
