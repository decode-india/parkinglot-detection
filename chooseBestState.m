function [bestState, bestMaxPotential] = chooseBestState(potentialFunction)

    % Get Best Potential Function
    bestRow = -Inf;
    bestCol = -Inf;
    bestAngle = -Inf; % degrees
    bestMaxPotential = -Inf;
    [~,~,numAngles] = size(potentialFunction);
    for i = 1:numAngles
        maxPotential = max(max(potentialFunction(:,:,i)));
        [row, col] = find( potentialFunction(:,:,i) >= maxPotential, 1, 'first' );
        if maxPotential > bestMaxPotential
            bestRow = row;
            bestCol = col;
            bestAngle = i;
            bestMaxPotential = maxPotential;
        end
    end % for


    noFeasibleStates = bestRow == -Inf && bestCol == -Inf && bestAngle == -Inf;
    if noFeasibleStates
        bestState = [];
    else
        bestState = [bestRow, bestCol, bestAngle];
    end
end % function
