function plotPointCloud(pointCloudObj, titleString)
    showPointCloud(pointCloudObj);
    colormap(parula)
    title(titleString);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
end %function
