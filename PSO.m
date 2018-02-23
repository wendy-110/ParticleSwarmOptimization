function out= PSO(problem, params)

    %% Problem Definition

    CostFunction=problem.CostFunction;     % Cost Function

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
    
    % The Flag for Showing Iteration Information
    ShowIterInfo=params.ShowIterInfo;   
    
    MaxVelocity=0.02*(VarMax-VarMin);
    MinVelocity=-MaxVelocity;
    
    %% Initialization

    % The Particle Template
    empty_particle.Position=[];
    empty_particle.Velocity=[];
    empty_particle.Cost=[];
    empty_particle.Best.Position=[];
    empty_particle.Best.Cost=[];

    % Create Population Array
    particle=repmat(empty_particle,nPop,1);

    % Initialize Global Best
    GlobalBest.Cost=inf;

    % Initialize Population Members
    for i=1:nPop
        % Generate Random Solution
        particle(i).Position=unifrnd(VarMin,VarMax,VarSize);

        % Initialize Velocity
        particle(i).Velocity=zeros(VarSize);

        % Evaluation
        particle(i).Cost=CostFunction(particle(i).Position);

        % Update the Personal Best
        particle(i).Best.Position=particle(i).Position;
        particle(i).Best.Cost=particle(i).Cost;

        % Update Global Best
        if particle(i).Best.Cost < GlobalBest.Cost
            GlobalBest=particle(i).Best;
        end

    end

    % Array to Hold Best Cost Value on Each Iteration
    BestCosts=zeros(MaxIt,1);
    
    % Array to hold history of position and velocities over time
    history = zeros(nPop,2*nVar,MaxIt);

%% Main Loop of PSO

% Set up video and axis limits
F(MaxIt) = struct('cdata',[],'colormap',[]);
xlim=[-1,1]; %[m] 
ylim=[-1,1]; %[m]
zlim=[-0.5,1];

% 2-D plotting
[X,Y] = meshgrid(linspace(-1,1));
Z = -4*sin(2*pi*X).*cos(1.5*pi*Y).*(1-X^2).*Y.*(1-Y);

% Main PSO Loop
    for it=1:MaxIt
        for i=1:nPop
            % Update Velocity
            particle(i).Velocity=w*particle(i).Velocity...
                +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position)...
                +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);

            % Apply Velocity Limits
            particle(i).Velocity=max(particle(i).Velocity, MinVelocity);
            particle(i).Velocity=min(particle(i).Velocity, MaxVelocity);
            
            % Update Position
            particle(i).Position=particle(i).Position+particle(i).Velocity;

            % Apply Lower and Upper Bound Limits
            particle(i).position=max(particle(i).Position, VarMin);
            particle(i).position=min(particle(i).Position, VarMax);
            
            % Evaluation
            particle(i).Cost=CostFunction(particle(i).Position);

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
        
        % Plot particles
        xData = [];
        yData = [];
        zData = [];
        xvData = [];
        yvData = [];
        for k = 1:nPop
            xData = [xData; particle(k).Position(1)];
            yData = [yData; particle(k).Position(2)];
            zData = [zData; particle(k).Position(3)];
            xvData = [xData; particle(k).Velocity(1)];
            yvData = [yData; particle(k).Velocity(2)];
        end
        fplot = scatter3(xData,yData,zData,'o');
        hold on
        contour(X,Y,Z)
        axis([xlim ylim zlim])
        hold off
        
        %make movie
        F(it) = getframe;
        
        % Update Movement history
        history(:,1,it) = xData;
        history(:,2,it) = yData;
        history(:,3,it) = xvData(2:end);
        history(:,4,it) = xvData(2:end);
        
        % Damping Inertia Coefficient
        w=w*wdamp;
    end
    
    %play movie
    movie(F,1,10)
    
    out.pop=particle;
    out.BestSol=GlobalBest;
    out.BestCosts=BestCosts;
    
v = VideoWriter('PSO3D.avi','Motion JPEG AVI');
v.FrameRate = 10;
open(v)
writeVideo(v,F)
close(v)
end