function [integral] = integrator(prev_integral, current_val, prev_val, timestep)
    % INTEGRATOR Compute discrete-time integral using trapezoidal rule
    %
    % Inputs:
    %   prev_integral - Previous integral value
    %   current_val   - Current input value
    %   prev_val      - Previous input value
    %   timestep      - Sampling time [s]
    %
    % Output:
    %   integral      - Updated integral value
    %
    % Uses trapezoidal (Tustin) integration method
    
    integral = prev_integral + (current_val + prev_val) * (timestep / 2);
end