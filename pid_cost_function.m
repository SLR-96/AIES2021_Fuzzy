%%%%%%%%%% Takes in the parameters optimized by the genetic algorithm and
%%%%%%%%%% uses them in FLC rules and system output gains. Then simulates
%%%%%%%%%% the system in a predetermined trajectory to calculate the errors
%%%%%%%%%% and use the sum of their absolute value as the cost.
function cost = pid_cost_function(p)

global NFE; % Number of Function Evaluations
NFE=NFE+1;

% PID gains need to be integer
P = floor(p(1:3));
I = floor(p(4:6));
D = floor(p(7:9));

%%%%%%%%%% Time discretization (Simulation time)
dt=0.01; ft=7;
time=0:dt:ft;
N=length(time);

%%%%%%%%%% Desired Trajectory
traj=load('xdes.mat');
x_des=traj.x_des;

%%%%%%%%%% Preallocating the Variables
x=zeros(N,6); tau=zeros(N,3); e=zeros(N,6);

%%%%%%%%%% Solving with Euler's Method
for i=1:N-1
    x(i+1,:)=x(i,:)+dt*plant_3r(x(i,:), tau(i,:));
    
    e(i,:)=x_des(i,:)-x(i,:);
    
    for j=1:6
        if e(i,j)>=10000 || isnan(e(i,j))
            e(i,j)=10000;
        elseif e(i,j)<=-10000
            e(i,j)=-10000;
        end
    end
    
    
    tau(i+1, :) = P.*e(i,1:2:5)+dt*I.*[sum(e(:,1)) sum(e(:,2)) ...
                sum(e(:,3))]+D.*e(i,2:2:6) + [0 (0.7/2*cos(x(i,4)+x(i,6))...
                +0.8*cos(x(i,4)))*9.81+0.8/2*cos(x(i,4))*9.81 ...
                0.7/2*cos(x(i,4)+x(i,6))*9.81];
end

e(N,:)=x_des(N,:)-x(N,:);

for j=1:6
    if e(N,j)>=10000 || isnan(e(N,j))
        e(N,j)=10000;
    elseif e(N,j)<=-10000
        e(N,j)=-10000;
    end
end

cost = sum(sum(abs(e)))+0.01*sum([P I D]);

end