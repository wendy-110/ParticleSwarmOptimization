clc;
clear;
close all;

%% Problem Definition

problem.CostFunction=@(x,y) CostFunction2(x,y);    % Cost Function
problem.nVar=3;                        % Number of Unknowns (Decision) Variables
problem.VarMin=-1;                     % Lower Bound of Decision Variables
problem.VarMax=1;                      % Upper Bound of Decision Variables

%% Parameter of PSO

% Constriction Coefficients
kappa=1;
phi1=0.5;
phi2=1.0;
phi=phi1+phi2;
chi=2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt=100;              % Maximum Number of Iterations
params.nPop=2;                 % Population Size (Swarm Size)
params.w=chi;                   % Inertia Coefficient
params.wdamp=1;                 % Damping Ratio of Inertia Weight
params.c1=chi*phi1;             % Personal Acceleration Coefficient
params.c2=chi*phi2;             % Social Acceleration Coefficient
params.ShowIterInfo=true;       % Flag for Showing Iteration Information

params.nTarg = 5;
params.detectionDist = 0.2;

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
