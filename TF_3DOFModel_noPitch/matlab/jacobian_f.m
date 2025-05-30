function F = jacobian_f(x, u, Ts)
    % Jacobiano della dinamica rispetto a x
    derB_h_dot = -u(1)*cos(x(2))*Ts + u(2)*sin(x(2))*Ts;
    F = [1, derB_h_dot;
         0, 1];
end