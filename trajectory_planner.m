function [theta, omega, alpha] = trajectory_planner(t, T, thi, tho)
%t is the current time
%T is the toal time of the movement. 
%thi is the initial angle
%tho is the target angle
% blend: Initial and final period in which acceleration is not constant

blend = floor(0.5*T);
V_max = (tho-thi)/(T-blend);
a = V_max/blend;
N = length(t);
alpha = zeros(N,1); omega = zeros(N,1); theta = zeros(N,1);

for i=1:N
    if (t(i)-t(1))<=blend
        alpha(i) = a;
        omega(i) = a*(t(i)-t(1));
        theta(i) = 1/2*a*(t(i)-t(1)).^2+thi;
    elseif (t(i)-t(1))<=(T-blend)
        alpha(i) = 0;
        omega(i) = V_max;
        theta(i) = 1/2*a*blend^2+thi+V_max.*(t(i)-t(1)-blend);
    elseif (t(i)-t(1))<=T
        alpha(i) = -a;
        omega(i) = V_max-a.*(t(i)-t(1)-(T-blend));
        theta(i) = -1/2*a.*(t(i)-t(1)-(T-blend)).^2+V_max*(t(i)-t(1)-(T-blend))...
                   +1/2*a*blend^2+thi+V_max*(T-2*blend);
    else
        alpha(i) = 0;
        omega(i) = 0;
        theta(i) = tho;
    end
end
end