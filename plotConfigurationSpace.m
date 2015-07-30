function plotConfigurationSpace(imageStack, titlestringFunc)
    if nargin == 1
        titlestringFunc = @(i) num2str(i);
    end
    numAngles = size(imageStack, 3);
    plotRows = floor(numAngles^0.5);
    plotCols = ceil(numAngles/plotRows);
    for i = 1:numAngles
        subplot(plotRows,plotCols,i)
        % imshow(imageStack(:,:,i));
        imshow(imageStack(:,:,i), [], 'Colormap', parula);
        title(titlestringFunc(i));
    end
end % function
