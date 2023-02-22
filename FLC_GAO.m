%%%%%%%%%% Genetic algorithm optimization
clc;
clear;
close all;
warning('off');
tic

%% Problem Definition

param = load('Parameters'); % Load parameters from a former run

global NFE; % Number of Function Evaluations
NFE=0;

CostFunction=@(x) flc_cost_function(x);     % Cost Function

nVar=36;  % Number of Decision Variables

%%%%%%%%%% Defining lower and upper bounds
% For rules choosing output mfs
VarMin = ones(1,27);
VarMax = 9.9999*ones(1,27);
% For weights
VarMin = [VarMin zeros(1,9)];
VarMax = [VarMax 100000*ones(1,3) 2*ones(1,6)];


%% GA Parameters

MaxIt=1000;      % Maximum Number of Iterations

nPop=100;        % Population Size

pc=0.8;                 % Crossover Percentage
nc=2*round(pc*nPop/2);  % Number of Offsprings (Parnets)

pm=1;                 % Mutation Percentage
nm=round(pm*nPop);      % Number of Mutants

gamma=0.05;      % Offspring Mutation Rate

mu=0.02;         % Mutation Rate

beta=8;         % Selection Pressure

%% Initialization

empty_individual.Position=[];
empty_individual.Cost=[];

% pop=repmat(empty_individual,nPop,1);
pop = param.pop;
for i=1:nPop
    
%       Initialize Position
%       pop(i).Position=unifrnd(VarMin,VarMax);
      
%     Evaluation
%     Parameters of the ith person in population
      pop(i).Cost=CostFunction(pop(i).Position);
    
end

% Sort Population
Costs=[pop.Cost];
[Costs, SortOrder]=sort(Costs);
pop=pop(SortOrder);

% Store Best Solution
BestSol=pop(1);

% Array to Hold Best Cost Values
BestCost=zeros(MaxIt,1);

% Store Cost
WorstCost=pop(end).Cost;

% Array to Hold Number of Function Evaluations
nfe=zeros(MaxIt,1);


%% Main Loop

for it=1:MaxIt
    
    % Calculate Selection Probabilities
    P=exp(-beta*Costs/WorstCost);
    P=P/sum(P);
    
    % Crossover
    popc=repmat(empty_individual,nc/2,2); % Children Population
    for k=1:nc/2
        
        % Select Parents Indices
        i1=RouletteWheelSelection(P);
        i2=RouletteWheelSelection(P);

        % Select Parents
        p1=pop(i1);
        p2=pop(i2);
        
        % Apply Crossover
        [popc(k,1).Position, popc(k,2).Position]=...
            Crossover(p1.Position,p2.Position,gamma,VarMin,VarMax);
        
        % Evaluate Offsprings
        popc(k,1).Cost=CostFunction(popc(k,1).Position);
        popc(k,2).Cost=CostFunction(popc(k,2).Position);
        
    end
    popc=popc(:);
    
    
    % Mutation
    popm=repmat(empty_individual,nm,1);
    for k=1:nm
        
        % Select Parent
        i=randi([1 nPop]);
        p=pop(i);
        
        % Apply Mutation
        popm(k).Position=Mutate(p.Position,mu,VarMin,VarMax);
        
        % Evaluate Mutant
        popm(k).Cost=CostFunction(popm(k).Position);
        
    end
    
    % Create Merged Population
    pop=[pop
         popc
         popm];
     
    % Sort Population
    Costs=[pop.Cost];
    [Costs, SortOrder]=sort(Costs);
    pop=pop(SortOrder);
    
    % Update Worst Cost
    WorstCost=max(WorstCost,pop(end).Cost);
    
    % Truncation
    pop=pop(1:nPop);
    Costs=Costs(1:nPop);
    
    % Store Best Solution Ever Found
    BestSol=pop(1);
    
    % Store Best Cost Ever Found
    BestCost(it)=BestSol.Cost;
    
    % Store NFE
    nfe(it)=NFE;
    

    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': NFE = ' num2str(nfe(it))...
        ', Best Cost = ' num2str(BestCost(it))]);
    
    toc
    
end

%% Results

figure;
semilogy(nfe,BestCost,'LineWidth',2);
xlabel('NFE');
ylabel('Cost');

p = BestSol.Position;
save('Parameters', 'pop', 'p')

