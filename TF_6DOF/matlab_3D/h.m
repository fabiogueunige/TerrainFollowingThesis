function z = h(x, s, num_s, num_m, n)
    % Observation function of the predicted measuraments
    printDebug('       h output function\n');
    %% Definition
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;   
    
    %% Funzione di osservazione: z_k = h(x_k, u_k)
    z = zeros(num_m, 1);
    for j = 1:num_s
        if (n'*s(:,j)) == 0
            error('n and sensor %.0f are parallel', j);
        end
        % Value update
        z(j) = -(x(IND_H)) / (n' * s(:,j));
    end

    printDebug('y1 = %.2f | y2 = %.2f | y3 = %.2f | y4 = %.2f\n', z(1), z(2), z(3), z(4));
end