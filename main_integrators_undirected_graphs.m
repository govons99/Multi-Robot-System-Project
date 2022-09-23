%% Initialization

% utilities
clc
clear
close all

% changable parameters
T = 20;          % simulation time
dt = 0.05;        % dt between instant of time
tau_min = 0.1;    % lowerbound jump time interval
tau_max = 0.5;    % upperbound jump time interval
save_plots = 1;
Lf = load('Laplacians/case1_flow.txt');
Lj = load('Laplacians/case1_jump.txt');

% define number of agents
[N, ~] = size(Lj);

% time vectors
tspan = 0:dt:T;
dim = length(tspan);

% generate the vector of jumps: tjump s.t.
% tau_min < tspan(tjump(i+1)) - tspan(tjump(i)) < tau_max
b1 = fix(tau_min/dt);
b2 = fix(tau_max/dt)-1;
tjump = 1;
count = tjump;
for i = 1:dim
    % generate the new instant of jump
    count = count + b1 + randi(b2 - b1);
    if count > dim
        break
    end
    % add the new instant to the vectors of jumps
    tjump = [tjump(1:i); count];
end
dim_j = length(tjump);

% initial conditions of the agents
x0 = zeros(N, 1);
for i = 1:N
    x0(i) = i;
end

%% Simulations

x_f = zeros(N, dim);    % flow states
x_j = zeros(N, dim);    % jump states
x_u = zeros(N, dim);    % union states
x_f(:, 1) = x0;
x_j(:, 1) = x0;
x_u(:, 1) = x0;

j = 1;

% upperbound for stability of jump evolution
a_star = alpha_star(Lj);

% coupling strenght for jump component
alpha = a_star/4;
     
 
for i = 1:dim - 1
    
    % simultations
    [~, x_ode_f] = ode45(@(t, x) sys(x, t, Lf), [tspan(i) tspan(i+1)], x_f(:, i));
    [~, x_ode_j] = ode45(@(t, x) sys(x, t, Lj), [tspan(i) tspan(i+1)], x_j(:, i));
    [~, x_ode_u] = ode45(@(t, x) sys(x, t, Lf), [tspan(i) tspan(i+1)], x_u(:, i)); 
    
    [idx, ~] = size(x_ode_f);
    x_f(:, i+1) = x_ode_f(idx, :);
    
    [idx, ~] = size(x_ode_j);
    x_j(:, i+1) = x_ode_j(idx, :);
    
    [idx, ~] = size(x_ode_u);
    x_u(:, i+1) = x_ode_u(idx, :);
    
    % hybrid evolution
    if j <= dim_j && i == tjump(j)
        x_u(:, i+1) = (eye(N) - alpha*Lj)*x_u(:, i);
        j = j+1;
    end
    
end

%%  plots

plot_stuff(tspan, x_f, N, 'Flow', save_plots);
plot_stuff(tspan, x_j, N, 'Jump', save_plots);
plot_stuff(tspan, x_u, N, 'Union', save_plots);

