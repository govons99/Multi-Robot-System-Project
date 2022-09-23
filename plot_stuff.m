function plot_stuff(t, x, N, label, save_plots)

    figure();
    plot(t, x, 'linewidth', 2); hold on; grid;
    title(strcat(label, ' Evolutions'));
    xlabel('Time [s]');
    ylabel('Agents');
    for i = 1:N
        lab(i) = strcat('Agent', {' '}, num2str(i));
    end
    legend(lab, 'location', 'ne', 'numcolumns', 2);
    
    if save_plots
        if ~isfolder('Figures')
            mkdir('Figures');
        end
        saveas(gcf, strcat('Figures/', label), 'png');
        saveas(gcf, strcat('Figures/', label), 'fig');
        saveas(gcf, strcat('Figures/', label), 'epsc');
    end
end

