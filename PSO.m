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
    
    MaxVelocity=0.05*(VarMax-VarMin);
    MinVelocity=-MaxVelocity;
    
    %% Get Genetic Alg parameters
    aa = params.GAString(1);
    ba = params.GAString(2);
    ar = params.GAString(3);
    br = params.GAString(4);
    at = params.GAString(5);
    bt = params.GAString(6);
    
    %% Initialization

    % deltaT
    dt = 0.2; %[s]
    
    % The Particle Template
    empty_particle.Position=[];
    empty_particle.Velocity=[];
    empty_particle.Cost=[];
    empty_particle.Best.Position=[];
    empty_particle.Best.Cost=[];
    empty_particle.Fuel = params.particleFuel;
    empty_particle.Attraction = [];
    particle=repmat(empty_particle,nPop,1);
    sum_initSepDist = 0;
    sepDistPerIt = zeros(MaxIt,1);
    
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
%         targets(i).Position = unifrnd(VarMin+1,VarMax-1,VarSize);
%         targets(i).Position = unifrnd(-1,-0.75,VarSize);
        targets(i).Position = params.startingLocInfo_targ(:,:,i);
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
        
        % Get Initial Separation Distance from Targets
        dist = 0;
        for k = 1:nTarg
             dist = dist + norm(particle(i).Position-targets(k).Position,2);
        end
        sum_initSepDist = sum_initSepDist + dist;
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

% Plot starting config
xData = [];
yData = [];
for k = 1:nPop
    xData = [xData; particle(k).Position(1)];
    yData = [yData; particle(k).Position(2)];
end
scatter(xData,yData,'o','b')

% Main PSO Loop
for it=1:MaxIt
        for i=1:nPop
            % Get attraction vector
            particle(i).Attraction = AttractionFunctionGA(aa,ba,ar,br,at,bt,particle(i).Position,particle,targets,obstacles,params);
            normalizedVel = (w*particle(i).Velocity + particle(i).Attraction)/norm((w*particle(i).Velocity + particle(i).Attraction),2);
            prevVel = particle(i).Velocity;
            particle(i).Velocity = 0.8*MaxVelocity*normalizedVel;
            currentVel = particle(i).Velocity;
            
            % Energy, or "Fuel" Evaluation
            workDone = abs((1/2)*(dot(currentVel,currentVel) - dot(prevVel,prevVel))); % Assumes mass is 1.
            particle(i).Fuel = particle(i).Fuel - workDone;
            
            % Update Position if EnoughFuel
            if particle(i).Fuel > 0
                particle(i).Position=particle(i).Position+particle(i).Velocity;
            end
      
            % Cost Evaluation
            particle(i).Cost=CostFunction(particle(i).Position,targets,obstacles);
            
            % Update Target Found
            distSum = 0;
            for k = 1:nTarg
                if norm(particle(i).Position - targets(k).Position,2) < detectionDist
                    targets(k).Found = 1;
                    GlobalBest.Cost=Inf; %Resets simulation
                end
                % Get total distance from all Targets
                distSum = distSum + norm(particle(i).Position-targets(k).Position,2);
            end
            % Store the Total Target Separation Distance
            sepDistPerIt(it) = sepDistPerIt(it)+distSum;
            
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
        for k = 1:nPop
            xData = [xData; particle(k).Position(1)];
            yData = [yData; particle(k).Position(2)];
        end
        
        %Plot the swarm particles
%         fplot = scatter3(xData,yData,zData,'o','b');
        scatter(xData,yData,'o','b')
        hold on

        % Plot ComDistance Radius of Particles
%         for k = 1:nPop
%             th = 0:2*pi/20:2*pi;
%             xunit = params.comDist * cos(th) + particle(k).Position(1);
%             yunit = params.comDist * sin(th) + particle(k).Position(2);
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
        % Plot Target Boundary Area
        line([problem.VarMax,problem.VarMax],[problem.VarMin,problem.VarMax],'Color','k')
        line([problem.VarMin,problem.VarMin],[problem.VarMin,problem.VarMax],'Color','k')
        line([problem.VarMin,problem.VarMax],[problem.VarMin,problem.VarMin],'Color','k')
        line([problem.VarMin,problem.VarMax],[problem.VarMax,problem.VarMax],'Color','k')
        
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
%         pause
        % If cost == 0, save the iteration num
        if BestCosts(it) == 0
            break;
        end
        
end

    %play movie
%     movie(F,1,10)
    

    out.endIt = it;
    out.pop=particle;
    out.BestSol=GlobalBest;
    out.BestCosts=BestCosts;
    out.NormalizedCost = sum(sepDistPerIt) / (it*sum_initSepDist);
    out.F = F; % save video
    
    
% v = VideoWriter('PSO_GA.avi','Motion JPEG AVI');
% v.FrameRate = 10;
% open(v)
% writeVideo(v,F)
% close(v)
end
