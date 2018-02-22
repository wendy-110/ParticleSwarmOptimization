function J = CostFunction2(Position,targets)

% TEMPORARY TO TEST FUNCTION
% Position = particle(1).Position;

% Initialize cost to zero
J = 0;

% Find closest target, and calculate cost based on distance
nTarg = size(targets,1);
cost = 0;
dist = 0;
distBest = Inf;
staticCost = 10;

for i = 1:nTarg
    if targets(i).Found == 0
        cost = cost + staticCost;
    end

    % Find next closest target
    dist = norm(Position-targets(i).Position,2);
    if dist < distBest && targets(i).Found == 0
        distBest = dist;
        cost = cost + distBest;
    end
    
end

% Return total cost
J = cost;

end

