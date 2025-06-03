function H = jacobian_h(x, gamma, lambda)
    % Jacobian of the observation function with respect to x
    delta = [gamma - (x(2) - x(3));
             lambda - (x(2) - x(3))];

    val1 = (((x(1)*sin(delta(1))) / ((cos(delta(1)))^2)));
    val2 = (((x(1)*sin(delta(2))) / ((cos(delta(2)))^2)));
    
    H = [(1/(cos(delta(1)))), -val1, val1, 0;
         (1/(cos(delta(2)))), -val2, val2, 0;
         0, 0, 1, 0];
    
end