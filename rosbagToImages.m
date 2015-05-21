
% ----------------------------------------
% load Bag
% ----------------------------------------
bagFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-03-22-wheelchairtests'
bagfile = '2015-03-22-16-41-16';
% bagfile = '2015-03-22-16-37-50';
bagFullFile = fullfile(bagFolder, [bagfile '.bag']);

projectFolder = '/home/vgan/code/datasets/gan2015wheelchair';
outputFolder = fullfile(projectFolder, [bagfile]);

% [points, imgRGB] = rosbagToPointCloud(bagFullFile, outputFolder);
[points, imgRGB] = rosbagToPointCloud2(bagFullFile, outputFolder);

% p = pointCloud(points, 'Color', imgRGB);
% showPointCloud(p);

