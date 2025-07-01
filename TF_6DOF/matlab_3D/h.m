function z = h(x, s, num_s, num_m, wRt, n)
    % Observation function of the predicted measuraments
    fprintf('       h output function\n');
    %% Definition
    IND_H = 1;                  I_IND_U = 1;
    ALPHA = 2;                  I_IND_W = 2;
    BETA = 3;
    PHI = 4;      M_PHI = 5;
    THETA = 5;    M_THETA = 6;
    IND_P = 6;    M_IND_P = 7;  I_IND_P = 3;
    IND_Q = 7;    M_IND_Q = 8;  I_IND_Q = 4;    
    
    %% Funzione di osservazione: z_k = h(x_k, u_k)
    z = zeros(num_m, 1);
    for j = 1:num_s
        z(j) = (x(IND_H)) / (n' * s(:,j));
    end
    fprintf('y1 = %.2f | y2 = %.2f | y3 = %.2f | y4 = %.2f\n', z(1), z(2), z(3), z(4));

    z(M_PHI) = x(PHI);
    z(M_THETA) = x(THETA);
    z(M_IND_P) = x(I_IND_P);
    z(M_IND_Q) = x(I_IND_Q);
end