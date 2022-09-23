%% Initialization

% utilities
clc
clear
close all

% number of agents
N = 6;

% definition of the state
syms x xj y yj th thj v w t real
q = [x y th v w]';

% time vector
time = 200;
dt = 0.05;
tspan = 0:dt:time;
dim = length(tspan);
ode_opt = odeset('MaxStep', 0.5);
save_plots = 1;
show_video = 0;

% adjacency matrix
A = [0 1 0 1 0 1;
    1 0 1 0 0 0;
    0 1 0 1 0 1;
    1 0 1 0 1 0;
    0 0 0 1 0 1;
    1 0 1 0 1 0];

% parameters for the control
pvi = 2;
pwi = pvi;
dvi = 1;
dwi = dvi;

% initial conditions
x0 = [2 7 7 3 1 1]';
y0 = [5 5.5 3.5 2 3.5 5.5]';
th0 = [0 -pi/4 -pi/2 pi/4 pi/2 pi/4]';

% position displacement
delta_x = [2 1 -1 -2 -1 1]';
delta_y = [0 2 2 0 -2 -2]';

% delta-persistent excitation
psi = 2.5 + 4/pi*sin(2*t);

% vector fields
phi = [cos(th) sin(th)]';
phip = [-sin(th) cos(th)]';
f = [phi*v; w; zeros(2, 1)];
g = [zeros(3, 2); eye(2)];


%% Simulation

% initialization of the arrays
x_u = zeros(N, dim);
y_u = zeros(N, dim);
th_u = zeros(N, dim);
v_u = zeros(N, dim);
w_u = zeros(N, dim);

% apply initial conditions
for i = 1:N

    x_u(i, 1) = x0(i);
    y_u(i, 1) = y0(i);
    th_u(i, 1) = th0(i);

end

% tranforming symbolic equations if Matlab Functions
f_f = matlabFunction(f);
phi_f = matlabFunction(phi);
phip_f = matlabFunction(phip);
psi_f = matlabFunction(psi);

for k = 1:dim - 1

    for i = 1:N
        
        % save current variables
        xi = x_u(i, k);
        yi = y_u(i, k);
        thi = th_u(i, k);
        vi = v_u(i, k);
        wi = w_u(i, k);

        % evaluate the functions numerically
        fni = f_f(thi, vi, wi);
        phi_i = phi_f(thi);
        phip_i = phip_f(thi);

        [ez, et] = state_error(A, i, x_u(:, k), y_u(:, k), th_u(:, k), delta_x, delta_y);

        % compute the controls
        uvi = -dvi*vi - pvi*phi_i'*ez;
        uwi = -dwi*wi - pwi*et + psi_f(tspan(k))*phip_i'*ez;

        u = [uvi; uwi];

        % simulate the system
        [~, x_ode] = ode45(@(t, q) sys_uni(t, q, fni, g, u), [tspan(k) tspan(k+1)], [xi yi thi vi wi], ode_opt);

        % save the state fot the next iteration
        [idx, ~] = size(x_ode);
        x_u(i, k+1) = x_ode(idx, 1);
        y_u(i, k+1) = x_ode(idx, 2);
        th_u(i, k+1) = x_ode(idx, 3);
        v_u(i, k+1) = x_ode(idx, 4);
        w_u(i, k+1) = x_ode(idx, 5);

    end

end

%% Plots

plot_stuff2(tspan, x_u, y_u, th_u, N, 'Unicycle', save_plots, 1)

if show_video
    evolution_video
end
