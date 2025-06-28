function F = jacobian_f(x, u, Ts)
    % Jacobiano della dinamica rispetto a x
    val = -u(1)*cos((x(2)-x(3)))*Ts + u(2)*sin((x(2)-x(3)))*Ts;
    F = [1, val, -val, 0;
         0, 1, 0, 0;
         0, 0, 1, Ts;
         0, 0, 0, 1];
end