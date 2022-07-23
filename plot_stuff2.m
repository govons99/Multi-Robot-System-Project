function plot_stuff2(tspan, x, y, th, N, label, save_plots, draw_line)

    colors = ['#0072BD'; '#D95319'; '#EDB120'; '#4DBEEE'; '#77AC30'; '#A2142F'];
    for i = 1:N
        lab(i) = strcat('Agent', {' '}, num2str(i));
    end

    figure()
    
    drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,varargin{:});
    for i = 1:N
        plot(x(i, :), y(i, :), 'linewidth', 2, 'color', colors(i, :)); grid on; hold on;
    end
    for i = 1:N
        X = [x(i, end) x(i, end)+cos(th(i, end))];
        Y = [y(i, end) y(i, end)+sin(th(i, end))];
        drawArrow(X, Y, 'linewidth', 2, 'color', colors(i, :))
        plot(x(i, 1), y(i, 1), '*', 'MarkerSize', 6, 'color', colors(i, :));
        plot(x(i, end), y(i, end), 'o', 'markerSize', 6, 'color', '#959494');
    end
    if draw_line
        line(x(1:2, end), y(1:2, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x(2:3, end), y(2:3, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x(3:4, end), y(3:4, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x(4:5, end), y(4:5, end), 'linewidth', 2, 'color', '#a8a8a8');
        line(x(5:6, end), y(5:6, end), 'linewidth', 2, 'color', '#a8a8a8');
        line([x(1, end) x(6, end)], [y(1, end) y(6, end)], 'linewidth', 2, 'color', '#a8a8a8');
    end
    title(strcat(label, ' Evolutions'));
    legend(lab, 'location', 'ne', 'numcolumns', 2);
    
    figure()
    plot(tspan, th, 'linewidth', 2); grid; hold on;
    title(strcat(label, ' Evolutions of $\theta$'), 'interpreter', 'latex');
    legend(lab, 'location', 'se', 'numcolumns', 2);
        
    if save_plots
        if ~isfolder('Figures')
            mkdir('Figures');
        end
        saveas(gcf, strcat('Figures\', label), 'png');
        saveas(gcf, strcat('Figures\', label), 'fig');
    end
end

