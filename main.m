clc;
clear;
close all;

%% Problem Definition

problem.CostFunction=@(x,y,z) CostFunction2(x,y,z);    % Cost Function
problem.AttractionFunction = @(x,y,z,l,o) AttractionFunctionGA(x,y,z,l,o); % Attraction Function
problem.nVar=2;                        % Number of Dimensions
problem.VarMin=-5;                     % Lower Bound of Decision Variables
problem.VarMax=5;                      % Upper Bound of Decision Variables

%% Parameter of PSO

% Constriction Coefficients
kappa=1;
phi1=3;
phi2=1.2;
phi=phi1+phi2;
chi=2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt=120;               % Maximum Number of Iterations
params.nPop=10;                 % Population Size (Swarm Size)
params.w=chi*2.0;                 % Inertia Coefficient
params.wdamp=1;                 % Damping Ratio of Inertia Weight
params.c1=chi*phi1;             % Personal Acceleration Coefficient
params.c2=chi*phi2;             % Social Acceleration Coefficient
params.particleRadius = 0.2;    % Physical particle radius
params.ShowIterInfo=false;      % Flag for Showing Iteration Information
params.particleFuel = 1;        % Starting Potential Energy

params.nTarg = 35;               % Number targets
params.detectionDist = 0.1*problem.VarMax;     % Target detection distance
params.comDist = 0.15*problem.VarMax;

params.nObs = 0;                % Number obstacles
params.obsRadius = 0.15*problem.VarMax;        % Obstacle avoidance radius

%% Initialize Genetic Algorithm Parameters

% Number of Genetic Strings to implement
numGeneticStrings = 8;
allStrings = zeros(numGeneticStrings,9);

% Define all possible values of alpha and beta for attraction, repulsion,
% and target interactions.
a_attr = linspace(0.01,20,100);
b_attr = linspace(1,3,100);
a_repul = linspace(1,50,100);
b_repul = linspace(-1,-3,100);
a_targ = linspace(1,50,100);
b_targ = linspace(1,3,100);

% Populate strings with randomized variables
for i = 1:numGeneticStrings
    allStrings(i,1) = a_attr(randi(100));
    allStrings(i,2) = b_attr(randi(100));
    allStrings(i,3) = a_repul(randi(100));
    allStrings(i,4) = b_repul(randi(100));
    allStrings(i,5) = a_targ(randi(100));
    allStrings(i,6) = b_targ(randi(100));
end

% OLD VALUES FOR TEST PURPOSES
% for i = 1:numGeneticStrings
%     allStrings(i,1) = 0.1;
%     allStrings(i,2) = 1;
%     allStrings(i,3) = 1;
%     allStrings(i,4) = -1;
%     allStrings(i,5) = .1;
%     allStrings(i,6) = 1;
% end


%% Initialization of Target Points and Initial Points

% Create Target Positions
initialTargArray = unifrnd(problem.VarMin,problem.VarMax,[1,problem.nVar,params.nTarg]);
params.startingLocInfo_targ = initialTargArray;

% CREATE TWO TARGETS (TESTING)
% initialTargArray = zeros(1,2,params.nTarg);
% initialTargArray(:,:,1) = [-1.5,1];
% initialTargArray(:,:,2) = [1.5,1];
% params.startingLocInfo_targ = initialTargArray;

% Create Initial Positions using initialPoints function
radius = 1;
numConfigs = 8; %Make this an even number.
stpt = [0,-7]; %[x,y]
initialPointsArray = initialPoints(params.nPop,radius,numConfigs,stpt);

%% ONLY GENETIC ALGORITHM

% Initialization
numGenerations = 10;
history_allStrings = zeros(numGeneticStrings,9,numGenerations);
history_bestCosts = zeros(120,1,numGenerations);

for genidx = 1:numGenerations
    fprintf('Analyzing Generation Number %i\n',genidx);
    for k = 1:numGeneticStrings
        params.GAString = allStrings(k,1:6);
        disp(['String Number ' int2str(k) ' out of ' int2str(numGeneticStrings)])
        params.startingLocInfo_pop = initialPointsArray(:,:,end);
        out=PSO(problem,params);
        allStrings(k,7) = out.endIt; 
        allStrings(k,8) = out.NormalizedCost;%Minimum Iterations out of all Configs
%         allStrings(k,8) = minIndex; %Save the configuration index w/ lowest number of Iterations
%         allStrings(k,9) = mean(IterationsTaken); %Save average num iterations over entire string
    end
    history_allStrings(:,:,genidx) = allStrings;
    allStrings = GenAlgorithm(allStrings,a_attr,b_attr,a_repul,b_repul,a_targ,b_targ);
end
%% ONLY DIFFERENT STARTING POSITIONS

%% Pass results to genetic algorithm to reconstruct strings

% Initialization
numGenerations = 4;

% Run optimization vs configs for specified number of Generations
for genidx = 1:numGenerations
    fprintf('Analyzing Generation Number %i\n',genidx);
    for k = 1:numGeneticStrings
        params.GAString = allStrings(k,1:6);
        for i = 1:numConfigs
            disp(['Configuration ' int2str(i) ' out of ' int2str(numConfigs)])
            params.startingLocInfo_pop = initialPointsArray(:,:,i);
            out=PSO(problem,params);
            IterationsTaken(i) = out.endIt;
        end
        [minIters,minIndex] = min(IterationsTaken);
        allStrings(k,7) = minIters; %Minimum #Iterations out of all Configs
        allStrings(k,8) = minIndex; %Save the configuration index w/ lowest number of Iterations
        allStrings(k,9) = mean(IterationsTaken); %Save average num iterations over entire string
    end
    allStrings = GenAlgorithm(allStrings,a_attr,b_attr,a_repul,b_repul,a_targ,b_targ);
end

%% Results

figure
semilogy(out.BestCosts, 'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;

