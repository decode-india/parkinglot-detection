function plotState(state, wheelchairMaps, totalMap)
    wheelChair = wheelchairMaps{state(1), state(2), state(3)};
    assert( sum(totalMap(wheelChair) == 1) == 0, 'state is infeasible!' ) % no collisions

    [mapYSize, mapXSize] = size(totalMap);

    figure
    map3 = zeros(mapYSize, mapXSize);
    map3(:,:,1) = totalMap;
    map3(:,:,2) = wheelChair;
    map3(:,:,3) = totalMap;
    imshow(map3)
    str = sprintf('Chosen Wheelchair Parking Spot, %d th angle', state(3));
    title(str);
end % function
