%% Initialization

% utilities
clc
clear
close all

starting_time = clock;

% changable parameters
T = 200;        % simulation time
dt = 0.02;
tau_min = 0.1;  % lowerbound jump time interval
tau_max = 0.5;    % upperbound jump time interval
save_plots = 0;
show_video = 1;

% initial conditions
x0 = [2 7 7 3 1 1]';
y0 = [5 5.5 3.5 2 3.5 5.5]';
th0 = [0 -pi/4 -pi/2 pi/4 pi/2 pi/4]';

% parameters for the control
pvi = 2;
pwi = pvi;
dvi = 1;
dwi = dvi;
% pvi = 2;
% pwi = pvi;
% dvi = 1;
% dwi = dvi;

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
Af = [0 1 0 0 0 0;
    1 0 1 0 0 0;
    0 1 0 0 0 0;
    0 0 0 0 1 0;
    0 0 0 1 0 1;
    0 0 0 0 1 0];

% jump adjacency matrix
Aj = [0 1 0 0 0 1;
    1 0 0 0 0 0;
    0 0 0 1 0 0;
    0 0 1 0 0 0;
    0 0 0 0 0 1;
    1 0 0 0 1 0];

% define number of agents
[N, ~] = size(Aj);

% time vectors
tspan = 0:dt:T;
dim = length(tspan);
ode_opt = odeset('MaxStep', 0.5);

% indices corresponding to when a jump happens
dim_j = fix(dim/1.5);
tjump = sort(randi(dim, [dim_j 1]));

% tau_min < tspan(tjump(i+1)) - tspan(tjump(i)) < tau_max
b1 = fix(tau_min/dt)+1;
b2 = fix(tau_max/dt)-1;
for i = 1:dim_j-1
    
    % delete an index if    tau_min > tspan(tjump(i+1)) - tspan(tjump(i))
    if i < dim_j && tjump(i+1) - tjump(i) < b1
        tjump(i+1) = [];
        dim_j = dim_j - 1;
    end
    
    % add new index if      tau_max < tspan(tjump(i+1)) - tspan(tjump(i))
    if i < dim_j && tjump(i+1) - tjump(i) > b2
        new_index = tjump(i) + b1 + randi(b2-b1);
        tjump = [tjump(1:i); new_index; tjump(i+1:end)];
        dim_j = dim_j+1;
    end
    
end

%% Simulations

% initialization of the arrays
x_f = zeros(N, dim);    % flow states
x_j = zeros(N, dim);    % jump states
x_u = zeros(N, dim);    % union states

y_f = zeros(N, dim);    % flow states
y_j = zeros(N, dim);    % jump states
y_u = zeros(N, dim);    % union states

th_f = zeros(N, dim);    % flow states
th_j = zeros(N, dim);    % jump states
th_u = zeros(N, dim);    % union states

v_f = zeros(N, dim);    % flow states
v_j = zeros(N, dim);    % jump states
v_u = zeros(N, dim);    % union states

w_f = zeros(N, dim);    % flow states
w_j = zeros(N, dim);    % jump states
w_u = zeros(N, dim);    % union states

uv_vec_u = zeros(N, dim);
uw_vec_u = zeros(N, dim);

% aply the intial conditions
x_f(:, 1) = x0;
x_j(:, 1) = x0;
x_u(:, 1) = x0;

y_f(:, 1) = y0;
y_j(:, 1) = y0;
y_u(:, 1) = y0;

th_f(:, 1) = th0;
th_j(:, 1) = th0;
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
        xi_f = x_f(i, k);
        xi_j = x_j(i, k);
        xi_u = x_u(i, k);

        yi_f = y_f(i, k);
        yi_j = y_j(i, k);
        yi_u = y_u(i, k);
        
        thi_f = th_f(i, k);
        thi_j = th_j(i, k);
        thi_u = th_u(i, k);

        vi_f = v_f(i, k);
        vi_j = v_j(i, k);
        vi_u = v_u(i, k);

        wi_f = w_f(i, k);
        wi_j = w_j(i, k);
        wi_u = w_u(i, k);

        % evaluate the functions for the 3 graphs
        fn_f = f_f(thi_f, vi_f, wi_f);
        fn_j = f_f(thi_j, vi_j, wi_j);
        fn_u = f_f(thi_u, vi_u, wi_u);

        phi_nf = phi_f(thi_f);
        phi_nj = phi_f(thi_j);
        phi_nu = phi_f(thi_u);

        phip_nf = phip_f(thi_f);
        phip_nj = phip_f(thi_j);
        phip_nu = phip_f(thi_u);

        % compute the errors
        [ez_f, et_f] = state_error(Af, i, x_f(:, k), y_f(:, k), th_f(:, k), delta_x, delta_y);
        [ez_j, et_j] = state_error(Aj, i, x_j(:, k), y_j(:, k), th_j(:, k), delta_x, delta_y);
        [ez_u, et_u] = state_error(Af, i, x_u(:, k), y_u(:, k), th_u(:, k), delta_x, delta_y);

        % compute the controls
        uv_f = -dvi*vi_f - pvi*phi_nf'*ez_f;
        uv_j = -dvi*vi_j - pvi*phi_nj'*ez_j;
        uv_u = -dvi*vi_u - pvi*phi_nu'*ez_u;

        uw_f = -dwi*wi_f - pwi*et_f + psi_f(tspan(k))*phip_nf'*ez_f;
        uw_j = -dwi*wi_j - pwi*et_j + psi_f(tspan(k))*phip_nj'*ez_j;
        uw_u = -dwi*wi_u - pwi*et_u + psi_f(tspan(k))*phip_nu'*ez_u;

        u_f = [uv_f; uw_f];
        u_j = [uv_j; uw_j];
        u_u = [uv_u; uw_u];

        uv_vec_u(i,k+1) = uv_u;
        uw_vec_u(i,k+1) = uw_u;

        % simulate the system
        [~, x_ode_f] = ode45(@(t, q) sys_uni(t, q, fn_f, g, u_f), [tspan(k) tspan(k+1)], [xi_f yi_f thi_f vi_f wi_f], ode_opt);
        [~, x_ode_j] = ode45(@(t, q) sys_uni(t, q, fn_j, g, u_j), [tspan(k) tspan(k+1)], [xi_j yi_j thi_j vi_j wi_j], ode_opt);
        [~, x_ode_u] = ode45(@(t, q) sys_uni(t, q, fn_u, g, u_u), [tspan(k) tspan(k+1)], [xi_u yi_u thi_u vi_u wi_u], ode_opt);

        % save the state fot the next iteration
        [idx, ~] = size(x_ode_f);
        x_f(i, k+1) = x_ode_f(idx, 1);
        y_f(i, k+1) = x_ode_f(idx, 2);
        th_f(i, k+1)= x_ode_f(idx, 3);
        v_f(i, k+1) = x_ode_f(idx, 4);
        w_f(i, k+1) = x_ode_f(idx, 5);

        [idx, ~] = size(x_ode_j);
        x_j(i, k+1) = x_ode_j(idx, 1);
        y_j(i, k+1) = x_ode_j(idx, 2);
        th_j(i, k+1)= x_ode_j(idx, 3);
        v_j(i, k+1) = x_ode_j(idx, 4);
        w_j(i, k+1) = x_ode_j(idx, 5);

        [idx, ~] = size(x_ode_u);
        x_u(i, k+1) = x_ode_u(idx, 1);
        y_u(i, k+1) = x_ode_u(idx, 2);
        th_u(i, k+1)= x_ode_u(idx, 3);
        v_u(i, k+1) = x_ode_u(idx, 4);
        w_u(i, k+1) = x_ode_u(idx, 5);
    
        % hybrid evolution
%         if j(i) <= dim_j && k+1 == tjump(j(i))
        if mod(k,3) == 0

            % evaluate the error using x_u considering the jump graph
            [ez_u, et_u] = state_error(Aj, i, x_u(:, k), y_u(:, k), th_u(:, k), delta_x, delta_y);
            uv_u = -dvi*vi_u - pvi*phi_nu'*ez_u;
            uw_u = -dwi*wi_u - pwi*et_u + psi_f(tspan(k))*phip_nu'*ez_u;

            uv_vec_u(i,k+1) = uv_u;
            uw_vec_u(i,k+1) = uw_u;
            
            % discretize evolution of the unicycle
            x_u(i, k+1) = x_u(i, k) + dt*v_u(i, k)*cos(th_u(i, k));
            y_u(i, k+1) = y_u(i, k) + dt*v_u(i, k)*sin(th_u(i, k));
            th_u(i, k+1) = th_u(i, k) + dt*w_u(i, k);
            v_u(i, k+1) = v_u(i, k) + dt*uv_u;
            w_u(i, k+1) = w_u(i, k) + dt*uw_u;

            j(i) = j(i) + 1;

        end

    end
    
end

duration_time(starting_time);

%% Plots

plot_stuff2(tspan, x_f, y_f, th_f, N, 'Flow', save_plots, 0)
plot_stuff2(tspan, x_j, y_j, th_j, N, 'Jump', save_plots, 0)
plot_stuff2(tspan, x_u, y_u, th_u, N, 'Union', save_plots, 1)

if show_video
    evolution_video
end

% figure()
% hold on; grid on;
% title("uv")
% plot(tspan,uv_vec_u, 'linewidth', 2)
% legend('1', '2', '3', '4', '5', '6')
% 
% figure()
% hold on; grid on;
% title('uw')
% plot(tspan, uw_vec_u, 'linewidth', 2);
% legend('1', '2', '3', '4', '5', '6')

