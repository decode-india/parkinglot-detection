function makeFeasibleMap( occupancyMap )

    % ========================================
    % 1. Generate wheelchair : object at multiple angles
    % ========================================
    numAngles = 9; % odd to include zero degrees
    minAngle = -30; % degrees
    maxAngle = 30; % degrees
    angles = linspace(minAngle, maxAngle, numAngles);
    wheelchairsize = [31 21]; % make odd to use centre point as reference
    wheelchair = makeWheelchairShape(wheelchairsize, angles);

    % ========================================
    % Plot wheelchair
    % ========================================
    figure;
    titlestringFunc = @(i) sprintf('wheelchair: %d degrees', angles(i));
    plotConfigurationSpace(wheelchair, titlestringFunc);

    % ========================================
    % 2. Generate wheelchairBlurred : blurred version of wheelchair
    % ========================================
    sigma1 = 3;
    for i = 1:numAngles
        wc = wheelchair(:,:,i);
        wheelchairBlurred(:,:,i) = imgaussfilt(wc, sigma1);
    end 

    % ========================================
    % Plot wheelchairBlurred
    % ========================================
    figure;
    titlestringFunc = @(i) sprintf('wheelchairBlurred: %d degrees', angles(i));
    plotConfigurationSpace(wheelchairBlurred, titlestringFunc);

    % ========================================
    % 3. Generate occupancyMapBlurred : blurred version of occupancy map
    % ========================================
    sigma2 = 4;
    occupancyMapBlurred = imgaussfilt(double(occupancyMap), sigma2);

    figure;
    imshow(occupancyMap);
    title('Occupancy Map');

    figure;
    imshow(occupancyMapBlurred);
    title('Occupancy Map - Blurred');

    % ========================================
    % 4. Generate convolved
    % ========================================
    for i = 1:numAngles
        feasibleStates(:,:,i) = conv2(occupancyMapBlurred, wheelchairBlurred(:,:,i), 'same');
    end % for

    % ========================================
    % Plot convolved
    % ========================================
    figure;
    titlestringFunc = @(i) sprintf('feasibleStates: %d degrees', angles(i));
    plotConfigurationSpace(feasibleStates, titlestringFunc);

    % ========================================
    % Stats
    % ========================================
    percentLessThanOne = sum(feasibleStates(:) <= 1) / numel(feasibleStates)

end % function
