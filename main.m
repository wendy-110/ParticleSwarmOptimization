clc;
clear;
close all;

%% Problem Definition

problem.CostFunction=@(x,y,z) CostFunction2(x,y,z);    % Cost Function
problem.AttractionFunction = @(x,y,z,l,o) AttractionFunction2(x,y,z,l,o); % Attraction Function
problem.nVar=2;                        % Number of Dimensions
problem.VarMin=-2;                     % Lower Bound of Decision Variables
problem.VarMax=2;                      % Upper Bound of Decision Variables

%% Parameter of PSO

% Constriction Coefficients
kappa=1;
phi1=3;
phi2=1.2;
phi=phi1+phi2;
chi=2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt=120;               % Maximum Number of Iterations
params.nPop=10;                 % Population Size (Swarm Size)
params.w=chi*2;                 % Inertia Coefficient
params.wdamp=1;                 % Damping Ratio of Inertia Weight
params.c1=chi*phi1;             % Personal Acceleration Coefficient
params.c2=chi*phi2;             % Social Acceleration Coefficient
params.particleRadius = 0.2;    % Physical particle radius
params.ShowIterInfo=false;      % Flag for Showing Iteration Information
params.particleFuel = 1;      % Starting Potential Energy

params.nTarg = 20;               % Number targets
params.detectionDist = 0.2*problem.VarMax;     % Target detection distance

params.nObs = 0;                % Number obstacles
params.obsRadius = 0.15*problem.VarMax;        % Obstacle avoidance radius

%% Initialization of Target Points and Initial Points

% Create Target Positions
initialTargArray = unifrnd(problem.VarMin,problem.VarMax,[1,problem.nVar,params.nTarg]);
params.startingLocInfo_targ = initialTargArray;

% Create Initial Positions using initialPoints function
radius = 1;
numConfigs = 8; %Make this an even number.
stpt = [0,-4]; %[x,y]
initialPointsArray = initialPoints(params.nPop,radius,numConfigs,stpt);

%% Call PSO
for i = 1:numConfigs
    disp(['Configuration ' int2str(i) ' out of ' int2str(numConfigs)])
    params.startingLocInfo_pop = initialPointsArray(:,:,i);
    out=PSO(problem,params);
    IterationsTaken(i) = out.endIt;

%     BestSol(i)=out.BestSol;
%     BestCosts(i)=out.BestCosts; 
end
%% Calling PSO

out=PSO(problem, params);
IterationsTaken = out.endIt;
BestSol=out.BestSol;
BestCosts=out.BestCosts;


%% Results

figure;
semilogy(BestCosts, 'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
