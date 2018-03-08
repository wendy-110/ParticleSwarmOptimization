clc;
clear;
close all;

%% Problem Definition

problem.CostFunction=@(x,y,z) CostFunction2(x,y,z);    % Cost Function
problem.nVar=2;                        % Number of Unknowns (Decision) Variables
problem.VarMin=-1;                     % Lower Bound of Decision Variables
problem.VarMax=1;                      % Upper Bound of Decision Variables

%% Parameter of PSO

% Constriction Coefficients
kappa=1;
phi1=3;
phi2=1.2;
phi=phi1+phi2;
chi=2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt=200;              % Maximum Number of Iterations
params.nPop=10;                 % Population Size (Swarm Size)
params.w=chi*2;                   % Inertia Coefficient
params.wdamp=1;                 % Damping Ratio of Inertia Weight
params.c1=chi*phi1;             % Personal Acceleration Coefficient
params.c2=chi*phi2;             % Social Acceleration Coefficient
params.ShowIterInfo=true;       % Flag for Showing Iteration Information

params.nTarg = 10;               % Number targets
params.detectionDist = 0.1;     % Target detection distance

params.nObs = 8;                % Number obstacles
params.obsRadius = 0.2;        % Obstacle avoidance radius

%% Calling PSO

out=PSO(problem, params);

BestSol=out.BestSol;
BestCosts=out.BestCosts;

%% Results

figure;
semilogy(BestCosts, 'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
