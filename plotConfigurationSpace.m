function plotConfigurationSpace(imageStack, titlestringFunc)
    numAngles = size(imageStack, 3);
    plotRows = floor(numAngles^0.5);
    plotCols = ceil(numAngles/plotRows);
    for i = 1:numAngles
        subplot(plotRows,plotCols,i)
        imshow(imageStack(:,:,i));
        % imshow(imageStack(:,:,i), [], 'Colormap', jet(255));
        title(titlestringFunc(i));
    end
end % function
