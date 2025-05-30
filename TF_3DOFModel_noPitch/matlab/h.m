function z = h(x, gamma, lambda)
    % Funzione di osservazione: z_k = h(x_k, u_k)
    delta = [(gamma - x(2));
             (lambda - x(2))];
    
    z = [x(1)/(cos(delta(1)));
         x(1)/(cos(delta(2)))];
    fprintf('Otput predetto (h) y1: %.2f m | y2: %.2f\n', z(1), z(2));
end