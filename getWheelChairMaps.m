% % wheelchairMaps: numRows x numCols x numAngles cell array of (numRows x numCols) logical
% function wheelchairMaps = getWheelChairMaps(wheelchairShapeAngle, numRows, numCols)
%     numAngles = size(wheelchairShapeAngle, 3);
%     wheelchairMaps = cell(numRows, numCols, numAngles);
%     wheelchairHeight = size(wheelchairShapeAngle, 1);
%     wheelchairWidth  = size(wheelchairShapeAngle, 2);
%     halfWheelchairHeight = ceil(wheelchairHeight/2);
%     halfWheelchairWidth = ceil(wheelchairWidth/2);
% 
% 
%     % TODO this conv2 is the bottleneck. speed up.
%     for r = 1:numRows
%     for c = 1:numCols
%         toppadding = r - halfWheelchairHeight;
%         leftpadding = c - halfWheelchairWidth;
%         botpadding = numRows - wheelchairHeight - toppadding;
%         rightpadding = numCols - wheelchairWidth - leftpadding;
% 
%         if (toppadding < 0) | (leftpadding < 0) | (botpadding < 0) | (rightpadding < 0) 
%             wheelChair = zeros(numRows, numCols, numAngles);
%         else
%             wheelChair = padarray(wheelchairShapeAngle, [toppadding, leftpadding], 'pre');
%             wheelChair = padarray(wheelChair, [botpadding, rightpadding], 'post');
%         end % if
%     assert( size(wheelChair, 1) == numRows);
%     assert( size(wheelChair, 2) == numCols);
% 
%     for thetaIdx = 1:numAngles
%         wheelchairMaps{r,c,thetaIdx} = wheelChair(:,:,thetaIdx);
%     end % for
% 
%     end % for
%     end % for
% end % function

% wheelchairMaps: numRows x numCols x numAngles cell array of (numRows x numCols) logical

function wheelchairMaps = getWheelChairMaps(wheelchairShapeAngle, numRows, numCols)
    numAngles = size(wheelchairShapeAngle, 3);
    wheelchairMaps = cell(numRows, numCols, numAngles);

    % TODO this conv2 is the bottleneck. speed up.
    for thetaIdx = 1:numAngles
        thetaIdx
    for r = 1:numRows
    for c = 1:numCols
        wheelChair = zeros(numRows,numCols);
        wheelChair(r,c) = 1;
        wheelChair = conv2(wheelChair, wheelchairShapeAngle(:,:,thetaIdx), 'same');
        wheelChair = wheelChair ~= 0;
        wheelchairMaps{r,c,thetaIdx} = wheelChair;
    end % for
    end % for
    end % for
end % function

% function wheelchairMaps = getWheelChairMaps(wheelchairShapeAngle, numRows, numCols)
%     numAngles = size(wheelchairShapeAngle, 3);
%     wheelchairMaps = cell(numRows, numCols, numAngles);
% 
%     % TODO this conv2 is the bottleneck. speed up.
%     for thetaIdx = 1:numAngles
%         thetaIdx
% 
%         [R,C] = meshgrid(1:numRows,1:numCols);
%         zeroMap = zeros(numRows,numCols);
%         convolveSame = @(r,c) conv2(makeImpulseMap(r,c,numRows,numCols),wheelchairShapeAngle(:,:,thetaIdx),'same');
% 
%         wheelchairs = bsxfun(convolveSame, R, C);
%         for r = 1:numRows
%         for c = 1:numCols
%             wheelchairMaps{r,c,thetaIdx} = wheelchairs(r,c);
%         end % for
%         end % for
%     end % for
% end % function
% 
% function impulseMap = makeImpulseMap(r,c,numRows,numCols)
%     impulseMap = zeros(numRows,numCols);
%     impulseMap(r,c) = 1;
% end % function
