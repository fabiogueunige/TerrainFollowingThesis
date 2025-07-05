function z = h(x, s, num_s, num_m, n)
    % Observation function of the predicted measuraments
    fprintf('       h output function\n');
    %% Definition
    IND_H = 1;                  
    ALPHA = 2;                  
    BETA = 3;
    PHI = 4;        M_PHI = 5;    
    THETA = 5;      M_THETA = 6;  
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;     
    
    %% Funzione di osservazione: z_k = h(x_k, u_k)
    z = zeros(num_m, 1);
    for j = 1:num_s
        z(j) = -(x(IND_H)) / (n' * s(:,j));
    end

    z(M_PHI) = x(PHI);
    z(M_THETA) = x(THETA);
    
    % z(M_IND_P) = x(I_IND_P);
    % z(M_IND_Q) = x(I_IND_Q);

    fprintf('y1 = %.2f | y2 = %.2f | y3 = %.2f | y4 = %.2f\n', z(1), z(2), z(3), z(4));
    fprintf('phi = %.2f | theta = %.2f\n', z(M_PHI), z(M_THETA));
end