function plotState(state, wheelchairShapeAngle, totalMap)
    [mapYSize, mapXSize] = size(totalMap);
    wheelChair = zeros(mapYSize, mapXSize);
    wheelChair(state(1),state(2)) = 1;
    wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,state(3)), 'same');
    wheelChair = wheelChair ~= 0;
    assert( sum(totalMap(wheelChair) == 1) == 0, 'state is infeasible!' ) % no collisions


    figure
    map3 = zeros(mapYSize, mapXSize);
    map3(:,:,1) = totalMap;
    map3(:,:,2) = wheelChair;
    map3(:,:,3) = totalMap;
    imshow(map3)
    str = sprintf('Chosen Wheelchair Parking Spot, %d th angle', state(3));
    title(str);
end % function
