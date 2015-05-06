function [wheelchairShapeAngle] = makeWheelchairShape(wheelchairsize, angles)
    numAngles = size(angles, 2);

    padsize = [10 10]; % surrounding 0's. Larger allows for larger rotations
    wheelchairShape = padarray(ones(wheelchairsize(1),wheelchairsize(2)), padsize); % at 0 degrees 
    wheelchairShapeAngle = zeros(size(wheelchairShape,1),size(wheelchairShape,2),numAngles);
    for i = 1:numAngles
        wheelchairShapeAngle(:,:,i) = imrotate(wheelchairShape, angles(i), 'bicubic', 'crop');
    end % for
end % function
