function a_star = alpha_star(L)
% expression of equation (12)

    eigL = eig(L);
    a_star = 1000;
    
    N = max(size(eigL));
    for i = 1:N
        if eigL(i) > 1.e-3
            e = 2*real(eigL(i))/(abs(eigL(i))^2);
            if e < a_star
                a_star = e;
            end
        end
    end

end

