function duration_time(starting_time)

    ending_time = clock;
    execution_time = ending_time - starting_time;
    if execution_time(5) < 0
        execution_time(4) = execution_time(4) - 1;
        execution_time(5) = execution_time(5) + 60;
    end
    if execution_time(6) < 0
        execution_time(5) = execution_time(5) - 1;
        execution_time(6) = execution_time(6) + 60;
    end
    disp('Execution time duration')
    disp(strcat(num2str(execution_time(5)), ' minutes'));
    disp(strcat(num2str(execution_time(6)), ' seconds'));
end

