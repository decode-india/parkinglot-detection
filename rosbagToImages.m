
% ----------------------------------------
% load Bag
% ----------------------------------------
% bagFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-03-22-wheelchairtests';
bagFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-04-28-chairtables';
% bagfile = '2015-03-22-16-41-16';
% bagfile = '2015-03-22-16-37-50';

bagFilenames = filenamesInFolder(bagFolder, '.bag');
numBagFiles = size(bagFilenames, 1);

% projectFolder = '/home/vgan/code/datasets/gan2015wheelchair';
projectFolder = fullfile(bagFolder, 'matlab_extract');

for i = 1:1
    bagFullFile = fullfile(bagFolder, bagFilenames{i});
    outputFolder = fullfile(projectFolder, bagFilenames{i});
    rosbagToPointCloud2(bagFullFile, outputFolder);
end % for

% p = pointCloud(points, 'Color', imgRGB);
% showPointCloud(p);

