function [x, y, th, v, w] = sim_unicycles(state_c, PD, f, g, phi, phip, psi, A, i, delta_x, delta_y)

    x_c = state_c(1);
    y_c = state_c(2);
    th_c = state_c(3);
    v_c = state_c(4);
    w_c = state_c(5);

    pvi = PD(1);
    dvi = PD(2);
    pwi = PD(3);
    dwi = PD(4);

    fn = f(th_c, v_c, w_c);
    phi_n = phi(th_c_c);
    phip_n = phip(th_c_c);

    ez = zeros(2, 1);
    eth = 0;

    for j = 1:N

        if A(i, j)

            zi = [x_c - delta_x(i);
                y_c - delta_y(i)];
            zj = [xk(j) - delta_x(j);
                yk(j) - delta_y(j)];
            
            ez = ez + zi - zj;

            eth = eth + th_c - thk(j);

        end

    end

    uvi = -dvi*v_c - pvi*phi_n'*ez;
    uwi = -dwi*w_c - pwi*eth + psi(tspan(k))*phip_n'*ez;

    u = [uvi; uwi];

    [~, x_ode] = ode45(@(t, q) sys_uni(t, q, fn, g, u), [tspan(k) tspan(k+1)], [x_c y_c th_c v_c w_c], ode_opt);

    [idx, ~] = size(x_ode);
    x_uni(i, k+1) = x_ode(idx, 1);
    y_uni(i, k+1) = x_ode(idx, 2);
    th_uni(i, k+1) = x_ode(idx, 3);
    v_uni(i, k+1) = x_ode(idx, 4);
    w_uni(i, k+1) = x_ode(idx, 5);

end

