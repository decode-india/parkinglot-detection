
% ----------------------------------------
% load Bag
% ----------------------------------------
bagFolder = '/media/vgan/stashpile/datasets/gan2015wheelchair/2015-03-22-wheelchairtests'
% bagfile = '2015-03-22-16-41-16.bag';
bagfile = '2015-03-22-16-37-50.bag';
bagFullFile = fullfile(bagFolder, bagfile);

% bag = ros.Bag(bagFullFile);


% ----------------------------------------
% RGB
% ----------------------------------------
% rgbTopicName = '/camera/rgb/image_raw';
% imgTopic = bag.readAll(rgbTopicName);
% 
% firstFrame = imgTopic{1};
% imHeight = firstFrame.height;
% imWidth = firstFrame.width;
% colorChannels = 3;
% 
% im = imgTopic{1}.data;
% im2 = reshape(im,colorChannels,imWidth,imHeight);
% im2 = permute(im2,[3 2 1]);
% imshow(im2)

% ----------------------------------------
% Depth
% ----------------------------------------
% depthTopicName = '/camera/depth/image_rect';
% depthTopic = bag.readAll(depthTopicName);
% 
% firstFrame = depthTopic{1};
% imHeight = firstFrame.height;
% imWidth = firstFrame.width;
% integerChannels = 4;
% 
% depthImg8 = depthTopic{1}.data;
% depthImg8 = reshape(depthImg8, integerChannels, imWidth, imHeight);
% depthImg8 = permute(depthImg8,[3 2 1]);
% imshow(depthImg8(:,:,1:3))
% 
% depthImg32 = zeros(imHeight,imWidth, 'uint32');
% for r = 1:imHeight
% for c = 1:imWidth
%     intArray = squeeze(depthImg8(r,c,:));
%     depthImg32(r,c) = typecast(uint8(intArray), 'uint32');
% end
% end
% 
% int_max32 = 2143289344;
% depthImg32Filt = depthImg32;
% INVALID_POINT_VAL = 0; % 1069144867 is min
% depthImg32Filt(depthImg32 == int_max32) = INVALID_POINT_VAL;
% 
figure
imshow(depthImg32Filt, [], 'Colormap', parula);

depthPointCloud = depthToCloud(depthImg32Filt);
depthPointCloudFiltered = depthPointCloud;

figure
showPointCloud(depthPointCloud)


% ----------------------------------------
% Depth Points
% ----------------------------------------
depthPointsTopicName = '/camera/depth/points';
depthPointsTopic = bag.readAll(depthPointsTopicName);
