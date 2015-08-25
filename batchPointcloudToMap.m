
function [] = batchPointcloudToMap(pointcloudList, saveFolder)

    numFrames = size(pointcloudList,1);


    allGroundMaps = cell(1, numFrames);
    allObjectMaps = cell(1, numFrames);
    allOrigins = zeros(numFrames, 2);

    % read function
    for i = 1:numFrames 
        pointcloudRaw = pointcloudList{i};
        [groundMap, objectMap, origin] = pointcloudToObjectMap(pointcloudRaw);
        imwrite(groundMap, fullfile(saveFolder, ['groundMap' num2str(i) '.png']));
        imwrite(objectMap, fullfile(saveFolder, ['objectMap' num2str(i) '.png']));

        allGroundMaps{i} = groundMap;
        allObjectMaps{i} = objectMap;
        allOrigins(i, :) = origin;
    end % for

    save( fullfile(saveFolder, ['originGroundObject' '.mat']), 'allOrigins', 'allGroundMaps', 'allObjectMaps' );

end % function

function [groundMap, objectMap, origin] = pointcloudToObjectMap(pointcloudRaw)
    voxelGridSize = 0.05; % in metres
    ransacParams.floorPlaneTolerance = 0.02; % tolerance in m
    ransacParams.maxInclinationAngle = 30; % in degrees
    [pointCloudRotated, originPointcloud] = processPointCloud(pointcloudRaw, voxelGridSize, ransacParams);


    metresPerMapUnit = 0.05; % each pixel represents metresPerMapUnit metres
    groundThreshold = 0.1; % in metres
    [objectMapHist, groundMapHist, origin] = pointCloudToObjectMap(pointCloudRotated, originPointcloud, metresPerMapUnit, groundThreshold);
    [mapYSize,mapXSize] = size(objectMapHist);

    sigmaWall = 0.7;
    objectMap = imgaussfilt(objectMapHist, sigmaWall) ~= 0; % > 1 for noise robustness

    sigmaGround = 0.5;
    groundMap = imgaussfilt(groundMapHist, sigmaGround) ~= 0; 
    groundMap(objectMap) = false;

end % function
