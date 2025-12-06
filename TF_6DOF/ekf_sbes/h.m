function z = h(x, s, num_s, num_m, n)
    printDebug('       h output function\n');
    
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;   
    
    %% Measurement Prediction
    % Initialize measurement vector
    z = zeros(num_m, 1);
    
    % Compute predicted range for each sensor
    for j = 1:num_s
        % Check for parallel condition (singularity)
        if abs(n'*s(:,j)) < 1e-6
            error('n and sensor %.0f are parallel', j);
        end
        
        % Range measurement model: d = -h / (n' * s)
        % Negative sign because altitude h is measured from robot down to terrain
        z(j) = -(x(IND_H)) / (n' * s(:,j));
    end

    % Debug output for monitoring measurements
    printDebug('y1 = %.2f | y2 = %.2f | y3 = %.2f | y4 = %.2f\n', z(1), z(2), z(3), z(4));
end