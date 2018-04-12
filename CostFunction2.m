function J = CostFunction2(Position,targets,obstacles)

% TEMPORARY TO TEST FUNCTION
% Position = particle(1).Position;
% cost function should evaluate the closest particle and move there

% Initialize cost to zero
J = 0;

% Find closest target, and calculate cost based on distance
nTarg = size(targets,1);
nObs = size(obstacles,1);
costTarg = 0;
costObs = 0;
dist = 0;
distBest = Inf;
staticTargCost = 15;
closestFactor = 1;
flag = 0;

% Calculate target costs
for i = 1:nTarg
    if targets(i).Found == 0
        costTarg = costTarg + staticTargCost;
    end
    % Add up all distances to cost
    dist = norm(Position-targets(i).Position,2);
    if dist < distBest && targets(i).Found == 0
        distBest = dist*closestFactor;
        flag = 1; %Flag as 'yes' if a better target is found
    end
    if flag == 1 
        costTarg = costTarg + distBest;
    end
end

% Calculate Obstacle costs
for i = 1:nObs
    % Add up all distances to cost
    dist = norm(Position-obstacles(i).Position,2);
    if dist < obstacles(i).obsRadius 
        costObs = 500;
    end
end

% Return total cost
J = costTarg+costObs;

end

