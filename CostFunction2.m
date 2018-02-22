function J = CostFunction2(Position,targets)

% z=-4*sin(2*pi*x(1))*cos(1.5*pi*x(2))*(1-x(1)^2)*x(2)*(1-x(2));

% TEMPORARY TO TEST FUNCTION
% Position = particle(1).Position;

% Initialize cost to zero
J = 0;

% Find closest target, and calculate cost based on distance
nTarg = size(targets,1);
delBest = Inf;
del = 0;
for i = 1:nTarg
    del = norm(Position-targets(i).Position,2);
    if del < delBest
        delBest = del;
    end
end

% Return delBest
J = delBest;

end

