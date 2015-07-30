function plotPlaneAroundPointCloud(plane, pointCloudObj)
    p = pointCloudObj;
    a = plane(1);
    b = plane(2);
    c = plane(3);
    d = plane(4);
    numTicks = 100;
    xScale = linspace(p.XLimits(1),p.XLimits(2), numTicks);
    yScale = linspace(p.YLimits(1),p.YLimits(2), numTicks);
    zScale = linspace(p.ZLimits(1),p.ZLimits(2), numTicks);
    [xx,yy,zz] = meshgrid(xScale, yScale, zScale);
    isosurface(xx, yy, zz, a*xx+b*yy+c*zz+d, 0)
end % function
