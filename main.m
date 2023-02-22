%%%%%%%%%% Project_3R_Robot

close all
clear
clc

param1 = load('PID_Parameters');
param2 = load('Parameters');
p1 = param1.p;
p2 = param2.p;

% PID gains need to be integer
P = floor(p1(1:3));
I = floor(p1(4:6));
D = floor(p1(7:9));

% Output MF index needs to be integer
a = floor(p2(1:27));

k_out = [p2(28) p2(29) p2(30)]; % System input gains

k_in = p2(31:36);

%%%%%%%%%% Defining input MFs as functions
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

mfs = load('active_mfs'); %Vectors showing the activated MFs of each input

%%%%%%%%% Initial Location
X0=1.5; Y0=0; Z0=1.5;
x0=cart_pol(X0, Y0, Z0);

%%%%%%%%% Desired Location and travel time
Xf=0.5; Yf=0.5; Zf=1.7;
xf=cart_pol(Xf, Yf, Zf);
T=5;

%%%%%%%%%% Time discretization (Simulation time)
dt=0.01; ft=7;
time=0:dt:ft;
N=length(time);

%%%%%%%%%% Desired trajectory
[x1des, x2des, acc1] = trajectory_planner(time, T, x0(1), xf(1));
[x3des, x4des, acc2] = trajectory_planner(time, T, x0(2), xf(2));
[x5des, x6des, acc3] = trajectory_planner(time, T, x0(3), xf(3));
x_des = [x1des x2des x3des x4des x5des x6des];
% traj=load('xdes.mat');
% x_des=traj.x_des;

%%%%%%%%%% Preallocating the Variables
x_a=zeros(N,6); tau_a=zeros(N,3); e_a=zeros(N,6);
x_b=zeros(N,6); tau_b=zeros(N,3); e_b=zeros(N,6);

%%%%%%%%%% Solving with FLC
for i=1:N-1
    x_a(i+1,:)=x_a(i,:)+dt*plant_3r(x_a(i,:), tau_a(i,:));
    
    e_a(i,:)=x_des(i,:)-x_a(i,:);
    inp.e1 = e_1(e_a(i,1));
    inp.e2 = e_2(e_a(i,2));
    inp.e3 = e_3(e_a(i,3));
    inp.e4 = e_4(e_a(i,4));
    inp.e5 = e_5(e_a(i,5));
    inp.e6 = e_6(e_a(i,6));
    tau_a(i+1,:) = k_out.*output_generator(out_range, mfs, a, k_in, inp,... 
                   out1, out2, out3)+[0  (0.7/2*cos(x_a(i,4)+x_a(i,6))...
                   +0.8*cos(x_a(i,4)))*9.81+0.8/2*cos(x_a(i,4))*9.81 ...
                   0.7/2*cos(x_a(i,4)+x_a(i,6))*9.81];
end
e_a(N,:)=x_des(N,:)-x_a(N,:);
cost_a = sum(sum(abs(e_a)));

%%%%%%%%%% Solving with PID controller
for i=1:N-1
    x_b(i+1,:)=x_b(i,:)+dt*plant_3r(x_b(i,:), tau_b(i,:));
    
    e_b(i,:)=x_des(i,:)-x_b(i,:);
    
    tau_b(i+1, :) = P.*e_b(i,1:2:5)+dt*I.*[sum(e_b(:,1)) sum(e_b(:,2)) ...
                sum(e_b(:,3))]+D.*e_b(i,2:2:6) + [0 (0.7/2*cos(x_b(i,4)+x_b(i,6))...
                +0.8*cos(x_b(i,4)))*9.81+0.8/2*cos(x_b(i,4))*9.81 ...
                0.7/2*cos(x_b(i,4)+x_b(i,6))*9.81];
end
e_b(N,:)=x_des(N,:)-x_b(N,:);
cost_b = sum(sum(abs(e_b)));

%% Plotting the Results
figure
subplot(3,1,1); plot(time,x_des(:,1),'k--',time,x_a(:,1),'b',time,x_b(:,1),'r')
legend('Desired Trajectory', 'FLC Trajectory', 'PID Trajectory')
xlabel('Time'); ylabel('\theta_1'); title('\theta_1 Trajectory')
subplot(3,1,2); plot(time,x_des(:,3),'k--',time,x_a(:,3),'b',time,x_b(:,3),'r')
legend('Desired Trajectory', 'FLC Trajectory', 'PID Trajectory')
xlabel('Time'); ylabel('\theta_2'); title('\theta_2 Trajectory')
subplot(3,1,3); plot(time,x_des(:,5),'k--',time,x_a(:,5),'b',time,x_b(:,5),'r')
legend('Desired Trajectory', 'FLC Trajectory', 'PID Trajectory')
xlabel('Time'); ylabel('\theta_3'); title('\theta_3 Trajectory')

figure
subplot(3,1,1); plot(time,x_des(:,2),'k--',time,x_a(:,2),'b',time,x_b(:,2),'r')
legend('Desired Velocity', 'FLC Velocity', 'PID Velocity')
xlabel('Time'); ylabel('\theta_1_d_o_t'); title('\theta_1_d_o_t Trajectory')
subplot(3,1,2); plot(time,x_des(:,4),'k--',time,x_a(:,4),'b',time,x_b(:,4),'r')
legend('Desired Velocity', 'FLC Velocity', 'PID Velocity')
xlabel('Time'); ylabel('\theta_2_d_o_t'); title('\theta_2_d_o_t Trajectory')
subplot(3,1,3); plot(time,x_des(:,6),'k--',time,x_a(:,6),'b',time,x_b(:,6),'r')
legend('Desired Velocity', 'FLC Velocity', 'PID Velocity')
xlabel('Time'); ylabel('\theta_3_d_o_t'); title('\theta_3_d_o_t Trajectory')

figure
subplot(3,1,1); plot(time,tau_a(:,1),'b', time,tau_b(:,1),'r')
legend('FLC Torque', 'PID Torque')
xlabel('Time'); ylabel('\tau_1'); title('\tau_1 Over Time')
subplot(3,1,2); plot(time,tau_a(:,2),'b', time,tau_b(:,2),'r')
legend('FLC Torque', 'PID Torque')
xlabel('Time'); ylabel('\tau_2'); title('\tau_2 Over Time')
subplot(3,1,3); plot(time,tau_a(:,3),'b', time,tau_b(:,3),'r')
legend('FLC Torque', 'PID Torque')
xlabel('Time'); ylabel('\tau_3'); title('\tau_3 Over Time')

