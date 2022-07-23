vf = figure('name', 'VIDEO');

tit = sgtitle(vf, sprintf('Time: %.2f s',tspan(1)));
grid on; axis equal; hold on;
xlim([-2 8])
ylim([0 8])

drawArrow = @(x,y,varargin) quiver(x(1),y(1),x(2)-x(1),y(2)-y(1),0,varargin{:});

colors = ['#0072BD'; '#D95319'; '#EDB120'; '#4DBEEE'; '#77AC30'; '#A2142F'];

% changeble parameters
agent_size = 4;
arrow_size = 3;
arrow_length = 0.5;
pixel_skipped = 20;

a1 = animatedline('Marker', 'o', 'Markersize', agent_size, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(1, :));
a2 = animatedline('Marker', 'o', 'Markersize', agent_size, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(2, :));
a3 = animatedline('Marker', 'o', 'Markersize', agent_size, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(3, :));
a4 = animatedline('Marker', 'o', 'Markersize', agent_size, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(4, :));
a5 = animatedline('Marker', 'o', 'Markersize', agent_size, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(5, :));
a6 = animatedline('Marker', 'o', 'Markersize', agent_size, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(6, :));

for i = 1:pixel_skipped:dim

    if i+pixel_skipped < dim
        tit = sgtitle( sprintf('Time: %.2f s',tspan(i)));
    else
        tit = sgtitle( sprintf('Time: %.2f s',tspan(dim)));
    end


    p1 = [x_u(1, i), y_u(1, i)]';
    p2 = [x_u(2, i), y_u(2, i)]';
    p3 = [x_u(3, i), y_u(3, i)]';
    p4 = [x_u(4, i), y_u(4, i)]';
    p5 = [x_u(5, i), y_u(5, i)]';
    p6 = [x_u(6, i), y_u(6, i)]';

    f1 = drawArrow([x_u(1, i) x_u(1, i)+arrow_length*cos(th_u(1, i))], ...
        [y_u(1, i) y_u(1, i)+arrow_length*sin(th_u(1, i))], ...
        'linewidth', arrow_size, 'color', colors(1, :));
    f2 = drawArrow([x_u(2, i) x_u(2, i)+arrow_length*cos(th_u(2, i))], ...
        [y_u(2, i) y_u(2, i)+arrow_length*sin(th_u(2, i))], ...
        'linewidth', arrow_size, 'color', colors(2, :));
    f3 = drawArrow([x_u(3, i) x_u(3, i)+arrow_length*cos(th_u(3, i))], ...
        [y_u(3, i) y_u(3, i)+arrow_length*sin(th_u(3, i))], ...
        'linewidth', arrow_size, 'color', colors(3, :));
    f4 = drawArrow([x_u(4, i) x_u(4, i)+arrow_length*cos(th_u(4, i))], ...
        [y_u(4, i) y_u(4, i)+arrow_length*sin(th_u(4, i))], ...
        'linewidth', arrow_size, 'color', colors(4, :));
    f5 = drawArrow([x_u(5, i) x_u(5, i)+arrow_length*cos(th_u(5, i))], ...
        [y_u(5, i) y_u(5, i)+arrow_length*sin(th_u(5, i))], ...
        'linewidth', arrow_size, 'color', colors(5, :));
    f6 = drawArrow([x_u(6, i) x_u(6, i)+arrow_length*cos(th_u(6, i))], ...
        [y_u(6, i) y_u(6, i)+arrow_length*sin(th_u(6, i))], ...
        'linewidth', arrow_size, 'color', colors(6, :));

    addpoints(a1, p1(1), p1(2))
    addpoints(a2, p2(1), p2(2))
    addpoints(a3, p3(1), p3(2))
    addpoints(a4, p4(1), p4(2))
    addpoints(a5, p5(1), p5(2))
    addpoints(a6, p6(1), p6(2))

    pause(0.001);

    if i + pixel_skipped < dim

        delete(f1);
        delete(f2);
        delete(f3);
        delete(f4);
        delete(f5);
        delete(f6);

    end

end

        line(x_u(1:2, end), y_u(1:2, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x_u(2:3, end), y_u(2:3, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x_u(3:4, end), y_u(3:4, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x_u(4:5, end), y_u(4:5, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x_u(5:6, end), y_u(5:6, end), 'linewidth', 2, 'color', '#a8a8a8');
        line([x_u(1, end) x_u(6, end)], [y_u(1, end) y_u(6, end)], 'linewidth', 2, 'color', '#a8a8a8');