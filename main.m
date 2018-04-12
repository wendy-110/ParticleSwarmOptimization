clc;
clear;
close all;

%% Problem Definition

problem.CostFunction=@(x,y,z) CostFunction2(x,y,z);    % Cost Function
problem.AttractionFunction = @(x,y,z,l,o) AttractionFunction2(x,y,z,l,o); % Attraction Function
problem.nVar=2;                        % Number of Dimensions
problem.VarMin=-3;                     % Lower Bound of Decision Variables
problem.VarMax=3;                      % Upper Bound of Decision Variables

%% Parameter of PSO

% Constriction Coefficients
kappa=1;
phi1=3;
phi2=1.2;
phi=phi1+phi2;
chi=2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt=100;              % Maximum Number of Iterations
params.nPop=16;                 % Population Size (Swarm Size)
params.w=chi*2;                   % Inertia Coefficient
params.wdamp=1;                 % Damping Ratio of Inertia Weight
params.c1=chi*phi1;             % Personal Acceleration Coefficient
params.c2=chi*phi2;             % Social Acceleration Coefficient
params.particleRadius = 0.1;   % Physical particle radius
params.ShowIterInfo=true;       % Flag for Showing Iteration Information

params.nTarg = 10;               % Number targets
params.detectionDist = 0.07*problem.VarMax;     % Target detection distance

params.nObs = 0;                % Number obstacles
params.obsRadius = 0.15*problem.VarMax;        % Obstacle avoidance radius

%% Add parameterization of starting variables

startingLocInfo_targ = unifrnd(problem.VarMin,problem.VarMax,[1 problem.nVar]);

radius = 1;
numConfigs = 8; %Make this an even number.
stpt = [0,-4]; %[x,y]
initialPointsArray = initialPoints(params.nPop,radius,numConfigs,stpt);


for i = 1:numConfigs
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

%% Call PSO Loop for each starting configuration

%line --> oval --> circle;
% equation of a circle = x^2 + y^2 = r^2;
% equation of oval = x^2/r2^2 + y^2/r1^2 = r^2;
% equally space the particles along the equation of a line

%% Results

figure;
semilogy(BestCosts, 'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
