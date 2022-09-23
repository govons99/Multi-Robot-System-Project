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

% adjacency matrix
A = [0 1 0 0 0 1;
    1 0 1 0 0 0;
    0 1 0 1 0 0;
    0 0 1 0 1 0;
    0 0 0 1 0 1;
    1 0 0 0 1 0];

% parameters for the control
pvi = 18;
pwi = 3;
dvi = 20;
dwi = 20;

pvi = 2;
pwi = 2;
dvi = 1;
dwi = 1;

% initial conditions
x0 = [2 7 7 3 1 1]';
y0 = [5 5.5 3.5 2 3.5 5.5]';
th0 = [0 -pi/4 -pi/2 pi/4 pi/2 pi/4]';

% position displacement
delta_x = [2 1 -1 -2 -1 1]';
delta_y = [0 2 2 0 -2 -2]';

% delta-persistent excitation
%psi = 2.5 + 4/pi*sin(2*t);

psi = 2.5 + 1/(4*pi)*sin(8*t);

% vector fields
phi = [cos(th) sin(th)]';
f = [phi*v; w; zeros(2, 1)];
g = [zeros(3, 2); eye(2)];

%% Simulation

x_uni = zeros(N, dim);
y_uni = zeros(N, dim);
th_uni = zeros(N, dim);
v_uni = zeros(N, dim);
w_uni = zeros(N, dim);

for i = 1:N

    x_uni(i, 1) = x0(i);
    y_uni(i, 1) = y0(i);
    th_uni(i, 1) = th0(i);

end

f_f = matlabFunction(f);
phi_f = matlabFunction(phi);
phip_f = matlabFunction([-sin(th); cos(th)]);
psi_f = matlabFunction(psi);

for k = 1:dim - 1

    xk = x_uni(:, k);
    yk = y_uni(:, k);
    thk = th_uni(:, k);
    vk = v_uni(:, k);
    wk = w_uni(:, k);

    for i = 1:N

        xi = xk(i);
        yi = yk(i);
        thi = thk(i);
        vi = vk(i);
        wi = wk(i);

        fni = f_f(thi, vi, wi);
        phi_i = phi_f(thi);
        phip_i = phip_f(thi);

        ezi = zeros(2, 1);
        ethi = 0;

        for j = 1:N
            
            if A(i, j)
                
                zi = [xi - delta_x(i);
                    yi - delta_y(i)];
                zj = [xk(j) - delta_x(j);
                    yk(j) - delta_y(j)];
                
                ezi = ezi + zi - zj;

                ethi = ethi + thi - thk(j);

            end

        end

        uvi = -dvi*vi - pvi*phi_i'*ezi;
        uwi = -dwi*wi - pwi*ethi + psi_f(tspan(k))*phip_i'*ezi;
        %uwi = -dwi*wi - pwi*ethi;
        
        phip_i'*ezi

        u = [uvi; uwi];

        [~, x_ode] = ode45(@(t, q) sys_uni(t, q, fni, g, u), [tspan(k) tspan(k+1)], [xi yi thi vi wi], ode_opt);

        [idx, ~] = size(x_ode);
        x_uni(i, k+1) = x_ode(idx, 1);
        y_uni(i, k+1) = x_ode(idx, 2);
        th_uni(i, k+1) = x_ode(idx, 3);
        v_uni(i, k+1) = x_ode(idx, 4);
        w_uni(i, k+1) = x_ode(idx, 5);

    end

end

%% Plots

plot_stuff2(tspan, x_uni, y_uni, th_uni, N, '', 0, 1)

% figure()
% for i = 1:N
%     plot(x_uni(i, :), y_uni(i, :), 'linewidth', 2); grid on; hold on;
% end
% 
% plot(x0, y0, '*', 'MarkerSize', 4);
% plot(x_uni(:, end), y_uni(:, end), 'ko', 'markerSize', 4);
% legend('Agent 1', 'Agent 2', 'Agent 3', 'Agent 4', 'Agent 5', 'Agent 6', 'location', 'ne');
% 
% drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,varargin{:});
% 
% for i = 1:N
%     
%     X = [x_uni(i,end) x_uni(i,end)+cos(th_uni(i,end))];
%     Y = [y_uni(i,end) y_uni(i,end)+sin(th_uni(i,end))];
%     drawArrow(X,Y,'linewidth',2)
%     
% end
% 
% figure()
% plot(tspan, th_uni, 'linewidth', 2); grid; hold on;
% legend('Agent 1', 'Agent 2', 'Agent 3', 'Agent 4', 'Agent 5', 'Agent 6', 'location', 'se');











