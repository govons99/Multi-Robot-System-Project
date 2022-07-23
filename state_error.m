function [ez, et] = state_error(A, i, x, y, th, delta_x, delta_y)

    ez = zeros(2, 1);
    et = 0;

    for j = 1:length(A)

        if A(i, j)
    
            zi = [x(i) - delta_x(i);
                y(i) - delta_y(i)];
            zj = [x(j) - delta_x(j);
                y(j) - delta_y(j)];
            
            ez = ez + zi - zj;
    
            et = et + th(i) - th(j);
    
        end
    end

end

