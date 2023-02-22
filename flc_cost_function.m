%%%%%%%%%% Takes in the parameters optimized by the genetic algorithm and
%%%%%%%%%% uses them in FLC rules and system output gains. Then simulates
%%%%%%%%%% the system in a predetermined trajectory to calculate the errors
%%%%%%%%%% and use the sum of their absolute value as the cost.
function cost = flc_cost_function(p)

global NFE; % Number of Function Evaluations
NFE=NFE+1;

a = floor(p(1:27)); % Output MF index needs to be integer

k_out = [p(28) p(29) p(30)]; % System input gains

k_in = p(31:36);

%%

%%%%%%%%%% Defining input MFs as functions
e_1 = @(in) [trapmf(in, [-10000, -10000, -0.02, 0]),...
            trimf(in, [-0.02, 0, 0.02]),...
            trapmf(in, [0, 0.02, 10000, 10000])];

e_2 = @(in) [trapmf(in, [-10000, -10000, -0.02, 0]),...
            trimf(in, [-0.02, 0, 0.02]),...
            trapmf(in, [0, 0.02, 10000, 10000])];
        
e_3 = @(in) [trapmf(in, [-10000, -10000, -0.02, 0]),...
            trimf(in, [-0.02, 0, 0.02]),...
            trapmf(in, [0, 0.02, 10000, 10000])];
        
e_4 = @(in) [trapmf(in, [-10000, -10000, -0.02, 0]),...
            trimf(in, [-0.02, 0, 0.02]),...
            trapmf(in, [0, 0.02, 10000, 10000])];
        
e_5 = @(in) [trapmf(in, [-10000, -10000, -0.02, 0]),...
            trimf(in, [-0.02, 0, 0.02]),...
            trapmf(in, [0, 0.02, 10000, 10000])];
        
e_6 = @(in) [trapmf(in, [-10000, -10000, -0.02, 0]),...
            trimf(in, [-0.02, 0, 0.02]),...
            trapmf(in, [0, 0.02, 10000, 10000])];


%% Output MFs:

out_range = -1.5:0.01:1.5;

out1 = zeros(13,length(out_range));
out1(1,:) = trimf(out_range, [-1.5, -1.2, -0.9]);
out1(2,:) = trimf(out_range, [-1.2, -0.9, -0.6]);
out1(3,:) = trimf(out_range, [-0.9, -0.6, -0.3]);
out1(4,:) = trimf(out_range, [-0.6, -0.3, 0]);
out1(5,:) = trimf(out_range, [-0.3, 0, 0.3]);
out1(6,:) = trimf(out_range, [0, 0.3, 0.6]);
out1(7,:) = trimf(out_range, [0.3, 0.6, 0.9]);
out1(8,:) = trimf(out_range, [0.6, 0.9, 1.2]);
out1(9,:) = trimf(out_range, [0.9, 1.2, 1.5]);

out2 = out1;
out3 = out1;

%% Results:
%%%%%%%%%% Load the vectors showing the activated MFs in each rule
mfs = load('active_mfs');

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
    
%     for j=1:6
%         if e(i,j)>=10000 || isnan(e(i,j))
%             e(i,j)=10000;
%         elseif e(i,j)<=-10000
%             e(i,j)=-10000;
%         end
%     end
    
    % inp is a structure consisting of the MFs activated in the inputs
    inp.e1 = e_1(e(i,1));
    inp.e2 = e_2(e(i,2));
    inp.e3 = e_3(e(i,3));
    inp.e4 = e_4(e(i,4));
    inp.e5 = e_5(e(i,5));
    inp.e6 = e_6(e(i,6));
    tau(i+1,:) = k_out.*output_generator(out_range, mfs, a, k_in, inp,... 
                out1, out2, out3)+[0 (0.7/2*cos(x(i,4)+x(i,6))...
                +0.8*cos(x(i,4)))*9.81+0.8/2*cos(x(i,4))*9.81 ...
                0.7/2*cos(x(i,4)+x(i,6))*9.81];
end

e(N,:)=x_des(N,:)-x(N,:);

% for j=1:6
%     if e(N,j)>=10000 || isnan(e(N,j))
%         e(N,j)=10000;
%     elseif e(N,j)<=-10000
%         e(N,j)=-10000;
%     end
% end

cost = sum(sum(abs(e)));

end