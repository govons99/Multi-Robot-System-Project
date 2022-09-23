%% Initialization

% utilities
clc
clear
close all

starting_time = clock;

% changable parameters
T = 200;          % simulation time
dt = 0.05;        % dt between instant of time
tau_min = 0.1;    % lowerbound jump time interval
tau_max = 0.35;    % upperbound jump time interval
save_plots = 1;
show_video = 0;

% initial conditions
x0 = [2 7 7 3 1 1]';
y0 = [5 5.5 3.5 2 3.5 5.5]';
th0 = [0 -pi/4 -pi/2 pi/4 pi/2 pi/4]';

% parameters for the control
pvi = 2;
pwi = pvi;
dvi = 1;
dwi = dvi;

% position displacement
delta_x = [2 1 -1 -2 -1 1]';
delta_y = [0 2 2 0 -2 -2]';

% symbolic definition of the state
syms x y th v w t real
q = [x y th v w]';

% delta-persistent excitation
psi = 2.5 + 4/pi*sin(2*t);

% vector fields
phi = [cos(th) sin(th)]';
phip = [-sin(th) cos(th)]';
f = [phi*v; w; zeros(2, 1)];
g = [zeros(3, 2); eye(2)];

% flow adjacency matrix
A = [0 1 0 0 0 1;
    1 0 1 0 0 0;
    0 1 0 1 0 0;
    0 0 1 0 1 0;
    0 0 0 1 0 1;
    1 0 0 0 1 0];

% define number of agents
[N, ~] = size(A);

% time vectors
tspan = 0:dt:T;
dim = length(tspan);
ode_opt = odeset('MaxStep', 0.5);

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

%% Simulations

% initialization of the arrays
x_u = zeros(N, dim);
y_u = zeros(N, dim);
th_u = zeros(N, dim);
v_u = zeros(N, dim);
w_u = zeros(N, dim);

ez = zeros(2, N);

% uv_vec_u = zeros(N, dim);
% uw_vec_u = zeros(N, dim);

% apply the intial conditions
x_u(:, 1) = x0;
y_u(:, 1) = y0;
th_u(:, 1) = th0;

% tranforming symbolic equations if Matlab Functions
f_f = matlabFunction(f);
phi_f = matlabFunction(phi);
phip_f = matlabFunction(phip);
psi_f = matlabFunction(psi);

j = ones(N, 1);

for k = 1:dim - 1

    for i = 1:N

        % save current variables
        xi = x_u(i, k);
        yi = y_u(i, k);
        thi = th_u(i, k);
        vi = v_u(i, k);
        wi = w_u(i, k);

        ezi = ez(:, i);

        % evaluate the functions numerically for the 3 graphs
        f_n = f_f(thi, vi, wi);
        phi_n = phi_f(thi);
        phip_n = phip_f(thi);

        et = 0;

        % compute the error for theta
        for c = 1:N
            if A(i, c)
                et = et + thi - th_u(c, k);
            end
        end

        % compute the controls
        uv = -dvi*vi - pvi*phi_n'*ezi;
        uw = -dwi*wi - pwi*et + psi_f(tspan(k))*phip_n'*ezi;
        u_n = [uv; uw];

%         % save the control profiles
%         uv_vec_u(i,k+1) = uv;
%         uw_vec_u(i,k+1) = uw;

        % simulate the system
        [~, x_ode] = ode45(@(t, q) sys_uni(t, q, f_n, g, u_n), [tspan(k) tspan(k+1)], [xi yi thi vi wi], ode_opt);

        % save the state fot the next iteration
        [idx, ~] = size(x_ode);
        x_u(i, k+1) = x_ode(idx, 1);
        y_u(i, k+1) = x_ode(idx, 2);
        th_u(i, k+1)= x_ode(idx, 3);
        v_u(i, k+1) = x_ode(idx, 4);
        w_u(i, k+1) = x_ode(idx, 5);
    
        % hybrid evolution
        if j(i) <= dim_j && k == tjump(j(i))

            % evaluate the error using x_u considering the jump graph
            [ezi, et] = state_error(A, i, x_u(:, k), y_u(:, k), th_u(:, k), delta_x, delta_y);
            ez(:, i) = ezi;
            % update only the control of the linear part
            uv = -dvi*vi - pvi*phi_n'*ezi;

%             % save the control profiles
%             uv_vec_u(i,k+1) = uv;
            
            % update only the linear part, the angular one have alredy been
            % computed
            x_u(i, k+1) = x_u(i, k) + dt*v_u(i, k)*cos(th_u(i, k));
            y_u(i, k+1) = y_u(i, k) + dt*v_u(i, k)*sin(th_u(i, k));
            v_u(i, k+1) = v_u(i, k) + dt*uv;

            j(i) = j(i) + 1;

        end

    end
    
end

duration_time(starting_time);

%% Plots

plot_stuff2(tspan, x_u, y_u, th_u, N, 'Union', save_plots, 1)

if show_video
    evolution_video
end

