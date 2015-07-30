function [wheelchairShapeAngle] = makeWheelchairShape(wheelchairsize, angles)
    numAngles = size(angles, 2);

    % Pad rectangle with zero's. Larger allows for larger rotations
    % Needed to ensure all rotations have equal size, instead of relying on imrotate
    padsize = [10 10]; 
    wheelchairShape = padarray(ones(wheelchairsize(1),wheelchairsize(2)), padsize); % at 0 degrees 

    wheelchairShapeAngle = zeros(size(wheelchairShape,1),size(wheelchairShape,2),numAngles);
    for i = 1:numAngles
        wheelchairShapeAngle(:,:,i) = imrotate(wheelchairShape, angles(i), 'bicubic', 'crop');
        wheelchairShapeAngle(:,:,i) = wheelchairShapeAngle(:,:,i) ~= 0; % binarize
    end % for
end % function
