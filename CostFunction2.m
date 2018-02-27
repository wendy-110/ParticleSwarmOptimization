function J = CostFunction2(Position,targets)

% TEMPORARY TO TEST FUNCTION
% Position = particle(1).Position;
% cost function should evaluate the closest particle and move there

% Initialize cost to zero
J = 0;

% Find closest target, and calculate cost based on distance
nTarg = size(targets,1);
cost = 0;
dist = 0;
distBest = Inf;
staticCost = 10;
closestFactor = 1;
flag = 0;


for i = 1:nTarg
    if targets(i).Found == 0
        cost = cost + staticCost;
    end

    % Add up all distances to cost
    dist = norm(Position-targets(i).Position,2);
    if dist < distBest && targets(i).Found == 0
        distBest = dist*closestFactor;
        flag = 1;
    end
    
    if flag == 1 
        cost = cost + distBest;
    end
end

% Return total cost
J = cost;

end

