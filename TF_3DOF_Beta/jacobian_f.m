function F = jacobian_f(x, u, Ts, theta)
    % Jacobiano della dinamica rispetto a x
    derB_h_dot = -u(1)*cos(x(2)-theta)*Ts + u(2)*sin(x(2)-theta)*Ts;
    F = [1, derB_h_dot;
         0, 1];
end