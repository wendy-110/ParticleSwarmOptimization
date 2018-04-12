function out= PSO(problem, params)

    %% Problem Definition

    CostFunction=problem.CostFunction;     % Cost Function
    AttractionFunction=problem.AttractionFunction;
    nVar=problem.nVar;              % Number of Unknowns (Decision) Variables

    VarSize=[1 nVar];               % Matrix Size of Decision Variables

    VarMin=problem.VarMin;          % Lower Bound of Decision Variables
    VarMax=problem.VarMax;          % Upper Bound of Decision Variables

    %% Parameters of PSO

    MaxIt=params.MaxIt;             % Maximum Number of Iterations

    nPop=params.nPop;               % Population Size (Swarm Size)

    w=params.w;                     % Inertia Coefficient
    wdamp=params.wdamp;             % Damping Ratio of Inertia Weight
    c1=params.c1;                   % Personal Acceleration Coefficient
    c2=params.c2;                   % Social Acceleration Coefficient
    detectionDist = params.detectionDist; % Value for target radius
    
    % The Flag for Showing Iteration Information
    ShowIterInfo=params.ShowIterInfo;   
    
    MaxVelocity=0.03*(VarMax-VarMin);
    MinVelocity=-MaxVelocity;
    
    %% Initialization

    % deltaT
    dt = 0.2; %[s]
    
    % The Particle Template
    empty_particle.Position=[];
    empty_particle.Velocity=[];
    empty_particle.Cost=[];
    empty_particle.Best.Position=[];
    empty_particle.Best.Cost=[];
    empty_particle.BatteryLife = [];
    empty_particle.Attraction = [];
    particle=repmat(empty_particle,nPop,1);
    
    % Targets template
    nTarg = params.nTarg;
    empty_targ.Position = [];
    empty_targ.Found = 0;
    targets = repmat(empty_targ,nTarg,1);
    
    % Obstacles template
    nObs = params.nObs;
    empty_obs.Position = [];
    empty_obs.obsRadius = params.obsRadius;
    obstacles = repmat(empty_obs,nObs,1);
    
    
    % Initialize targets position
    for i = 1:nTarg
        targets(i).Position = unifrnd(VarMin+1,VarMax-1,VarSize);
%         targets(i).Position = unifrnd(-1,-0.75,VarSize);
    end
    
    % Initialize obstacles position
    for i = 1:nObs
        obstacles(i).Position = unifrnd(VarMin*0.6,VarMax*0.6,VarSize);
    end
    
    % Initialize Global Best
    GlobalBest.Cost=inf;

    % Initialize Population Members
    for i=1:nPop
        % Generate Random Solution
%         particle(i).Position=unifrnd(-1,-.70,VarSize);
        particle(i).Position=params.startingLocInfo_pop(i,:);

        % Initialize Velocity
        particle(i).Velocity=zeros(VarSize);

        % Evaluation
        particle(i).Cost=CostFunction(particle(i).Position,targets,obstacles);

        % Update the Personal Best
        particle(i).Best.Position=particle(i).Position;
        particle(i).Best.Cost=particle(i).Cost;

        % Update Global Best
        if particle(i).Best.Cost < GlobalBest.Cost
            GlobalBest=particle(i).Best;
        end

    end

    % Array to Hold Best Cost Value on Each Iteration
    BestCosts=inf(MaxIt,1);
    
    % Array to hold history of position and velocities over time
    history = zeros(nPop,2*nVar,MaxIt);

%% Main Loop of PSO

% Video settings initialization
F(MaxIt) = struct('cdata',[],'colormap',[]);
xlim=[problem.VarMin,problem.VarMax]; %[m] 
ylim=[problem.VarMin-3,problem.VarMax]; %[m]
% zlim=[-1.5,1.5];

% Main PSO Loop
for it=1:MaxIt
        for i=1:nPop
            % Get attraction vector
            particle(i).Attraction = AttractionFunction(particle(i).Position,particle,targets,obstacles,params);
            
            % Update Velocity, Euler approximation
            % Normalize Velocity
%             particle(i).Velocity = w*particle(i).Velocity + 0.06*(particle(i).Attraction/norm(particle(i).Attraction,2));

% particle(i).Velocity=w*particle(i).Velocity...
%                 +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position)...
%                 +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
             
            % Update Velocity and Apply Velocity Limits
%             particle(i).Velocity=max(particle(i).Velocity, MinVelocity);
%             particle(i).Velocity=min(particle(i).Velocity, MaxVelocity);
            normalizedVel = (w*particle(i).Velocity + particle(i).Attraction)/norm((w*particle(i).Velocity + particle(i).Attraction),2);
            particle(i).Velocity = MaxVelocity*normalizedVel;
            
            % Update Position
            particle(i).Position=particle(i).Position+particle(i).Velocity;

            % Apply Lower and Upper Bound Limits
            particle(i).Position=max(particle(i).Position, VarMin);
            particle(i).Position=min(particle(i).Position, VarMax);
            
            % Evaluation
            particle(i).Cost=CostFunction(particle(i).Position,targets,obstacles);
                        
            % Update Target Found
            for k = 1:nTarg
                if norm(particle(i).Position - targets(k).Position,2) < detectionDist
                    targets(k).Found = 1;
%                     fprintf('found target num %f\n',k);
                    GlobalBest.Cost=Inf; %Resets simulation
                end
            end

            % Update Personal Best
            if particle(i).Cost< particle(i).Best.Cost

                particle(i).Best.Position=particle(i).Position;
                particle(i).Best.Cost=particle(i).Cost;

                % Update Global Best
                if particle(i).Best.Cost<GlobalBest.Cost
                    GlobalBest=particle(i).Best;
                end

            end

        end

        % Store the Best Cost Value
        BestCosts(it)=GlobalBest.Cost;

        % Display Iteration Information
        if ShowIterInfo
            disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
        end
        
        % Arrange particle, target information into arrays
        xData = [];
        yData = [];
%         zData = [];
%         xvData = [];
%         yvData = [];
%         zvData = [];
        for k = 1:nPop
            xData = [xData; particle(k).Position(1)];
            yData = [yData; particle(k).Position(2)];
%             zData = [zData; particle(k).Position(3)];
%             xvData = [xvData; particle(k).Velocity(1)];
%             yvData = [yvData; particle(k).Velocity(2)];
%             zvData = [zvData; particle(k).Velocity(3)];
        end
        
        %Plot the swarm particles + circle of radius particleRadius
%         fplot = scatter3(xData,yData,zData,'o','b');
        scatter(xData,yData,'o','b')
        hold on
%         for k = 1:nPop
%             th = 0:2*pi/20:2*pi;
%             xunit = params.particleRadius * cos(th) + particle(k).Position(1);
%             yunit = params.particleRadius * sin(th) + particle(k).Position(2);
%             plot(xunit, yunit,'b');
%             hold on
%         end
        
        %Plot found and unfound targets
        foundCount = 1; %for indexing purposes
        unfoundCount = 1;
        targetsFound = [];
        targetsUnfound = [];
        for k = 1:nTarg
            if targets(k).Found == 1
                targetsFound(foundCount,:) = targets(k).Position;
                foundCount = foundCount + 1;
            else
                targetsUnfound(unfoundCount,:) = targets(k).Position;
                unfoundCount = unfoundCount + 1;
            end
        end
        if isempty(targetsFound) ~= 1
%             scatter3(targetsFound(:,1),targetsFound(:,2),targetsFound(:,3),'x','b')
            scatter(targetsFound(:,1),targetsFound(:,2),'x','b')
            hold on
        end
        if isempty(targetsUnfound) ~= 1
%             scatter3(targetsUnfound(:,1),targetsUnfound(:,2),targetsUnfound(:,3),'x','r')
            scatter(targetsUnfound(:,1),targetsUnfound(:,2),'x','r')            
            hold on
        end

        % Plot obstacles
        if nObs >=1
            obsTemp = [];
            for k = 1:nObs
                obsTemp(k,:) = obstacles(k).Position;
            end
             scatter(obsTemp(:,1),obsTemp(:,2),'s','r','filled')
            hold on
        end
       
        % Adjust axes
%         axis([xlim ylim zlim])
        axis([xlim ylim])
        hold off
        
        %make movie
        F(it) = getframe;
        
        % Update Movement history
%         history(:,1,it) = xData;
%         history(:,2,it) = yData;
%         history(:,3,it) = zData;
%         history(:,4,it) = xvData;
%         history(:,5,it) = yvData;
%         history(:,6,it) = zvData;
        
        % Damping Inertia Coefficient
        w=w*wdamp;
        
        % If cost == 0, save the iteration num
        if BestCosts(it) == 0
            break;
        end

end
    
    %play movie
    movie(F,1,10)
    
    out.endIt = it;
    out.pop=particle;
    out.BestSol=GlobalBest;
    out.BestCosts=BestCosts;
    
% v = VideoWriter('PSO_attraction_method.avi','Motion JPEG AVI');
% v.FrameRate = 10;
% open(v)
% writeVideo(v,F)
% close(v)
end
