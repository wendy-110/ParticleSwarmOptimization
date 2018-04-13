function sumForces = AttractionFunctionGA(aa,ba,ar,br,at,bt,Position,particle,targets,obstacles,params)

% Initialize total force to zero, get array sizes
sumForces = 0;
nTarg = size(targets,1);
nObs = size(obstacles,1);
nPart = size(particle,1);

% Get info from params
particleRad = params.particleRadius;
obstacleRad = params.obsRadius;

%For member-member interaction
a_attr = aa;
b_attr = ba;
a_repul = ar;
b_repul = br; %must be negative
F_mm = [0 0];
for i = 1:nPart
    %    Calculate acceleration + direction
    dist = norm(Position-particle(i).Position,2);
    direction = particle(i).Position-Position;
    % If particles too close, repel. Else, attract.
    if dist ~= 0
        if dist > particleRad
            F_mm = F_mm + (a_attr*dist^b_attr)*direction;
        elseif dist < particleRad
            F_mm = F_mm - (a_repul*dist^b_repul)*direction;
        end
    end
end

%For member-target interaction
a_targ = at;
b_targ = bt;
F_mt = [0 0];
lowestDist = Inf;
lowestDir = [0 0];
for i = 1:nTarg
    if targets(i).Found == 0
        dist = norm(Position-targets(i).Position,2);
        if dist < lowestDist
            lowestDist = dist;
            lowestDir = targets(i).Position-Position;
        end
        F_mt = F_mt+(a_targ*lowestDist^b_targ)*lowestDir;
    end
end

%For member-obstacle interaction
a_obs = 30;
b_obs = -1;% must be negative
F_mo = [0 0];
for i = 1:nObs
%    calculate acceleration + direction
    dist = norm(Position-obstacles(i).Position,2);
      if dist < obstacleRad
        direction = obstacles(i).Position-Position;
        F_mo = F_mo-(a_obs*dist^b_obs)*direction;
      end
end
% F_mo

% Return total cost
sumForces = F_mm + F_mt + F_mo;

end

