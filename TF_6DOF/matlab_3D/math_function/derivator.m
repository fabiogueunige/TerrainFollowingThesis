function [derivative] = derivator(prev_derivative, current_val, prev_val, timestep)
    % DERIVATOR Compute discrete-time derivative using trapezoidal rule
    %
    % Inputs:
    %   prev_derivative - Previous derivative value
    %   current_val     - Current input value
    %   prev_val        - Previous input value  
    %   timestep        - Sampling time [s]
    %
    % Output:
    %   derivative      - Computed derivative
    %
    % Uses a tustin discretization with gain parameter alpha = 1
    
    alpha = 1.0;  % Tustin discretization gain
    derivative = ((2 - alpha*timestep)*prev_derivative + ...
                  2*alpha*current_val - 2*alpha*prev_val) / (2 + alpha*timestep);
end