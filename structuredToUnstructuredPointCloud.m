% M x N x 3 to (N*M) x 3
function pointCloudDenoised = structuredToUnstructuredPointCloud(pointCloudObj)
    allIndicies = 1:pointCloudObj.Count;
    pointCloudDenoised = select(pointCloudObj, allIndicies);
end % function
