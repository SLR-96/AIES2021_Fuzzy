%%%%%%%%%% Prerequisites for designing the system
clear
%% Part 1: Fuzzy Rules
% Defines 6 vectors with the length of the rules count, to determine the
% MFs activated in each rule and then save them to a file


rule=1;
e1_active = zeros(1, 27);
e2_active = zeros(1, 27);
e3_active = zeros(1, 27);
e4_active = zeros(1, 27);
e5_active = zeros(1, 27);
e6_active = zeros(1, 27);

for a=1:3
    for b=1:3
        e1_active(rule)=a;
        e2_active(rule)=b;
        e3_active(rule+9)=a;
        e4_active(rule+9)=b;
        e5_active(rule+18)=a;
        e6_active(rule+18)=b;
        rule=rule+1;
    end
end
    
save('active_mfs', 'e1_active', 'e2_active', 'e3_active', 'e4_active',...
                       'e5_active', 'e6_active')

%% Part 2: Desired Trajectory
% Define a trajectory to be followed by the links in order to optimize the
% genetic algorithm parameters and save it to a file

%%%%%%%%%% Desired Location and travel time
x1=[0 0 0];
xf1=[2 1.5 1.5];
xf2=[-3 -2.36 -2.36];
T=5;

%%%%%%%%%% Time discretization (Simulation time)
t0=0; dt=0.01; ft1=7;
ft2=2*ft1; time1=t0:dt:ft1; time2=ft1+dt:dt:ft2;

%%%%%%%%%% Desired trajectory
blend=3; %Initial and final period in which acceleration is not constant
[x1des, x2des, acc1] = trajectory_planner(time1, T, x1(1), xf1(1));
[x3des, x4des, acc2] = trajectory_planner(time1, T, x1(2), xf1(2));
[x5des, x6des, acc3] = trajectory_planner(time1, T, x1(3), xf1(3));

[x1des2, x2des2, acc1] = trajectory_planner(time2, T, xf1(1), xf2(1));
[x3des2, x4des2, acc2] = trajectory_planner(time2, T, xf1(2), xf2(2));
[x5des2, x6des2, acc3] = trajectory_planner(time2, T, xf1(3), xf2(3));


x_des = [x1des  x2des  x3des  x4des  x5des  x6des];
%          x1des2 x2des2 x3des2 x4des2 x5des2 x6des2];

save('xdes')

