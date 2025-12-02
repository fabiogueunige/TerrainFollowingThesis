function [kp, ki, kd, kt] = gainComputation(speed0, dim_i)

    
    %% Control Design Parameters
    % Natural frequency and damping for pole placement
    wn = 0.5;       % Natural frequency for translation [rad/s]
    damp = 0.7;     % Damping ratio
    p = 12;         % Additional pole for PID controller
    
    % Parameters for angular control (roll, pitch, yaw)
    wn_angle = 1.5;      % Higher natural frequency for angles
    damp_angle = 0.8;    % Higher damping for angles
    p_angle = 15;        % Higher pole for angles

    % Initialize gain vectors
    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);
    Ti = zeros(dim_i, 1);   % Integral time constant
    Td = zeros(dim_i, 1);   % Derivative time constant
    kt = zeros(dim_i, 1);   % Anti-windup gain

    %% BlueROV2 Physical Parameters
    m = 11.5;   % Total mass [kg]
    I = diag([0.21, 0.245, 0.245]);  % Inertia tensor [kg*m^2]

    % Added mass coefficients (negative of acceleration-induced forces)
    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1]; 
    
    % Linear damping coefficients (proportional to velocity)
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5];
    
    % Quadratic damping coefficients (proportional to |velocity|*velocity)
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1];
    
    %% Effective Dynamics
    % Virtual mass (total mass + added mass)
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    % Dissipative forces linearized around operating point speed0
    % d = -tau_r - tau_d*|v| (nonlinear model)
    dv = -tau_r - tau_d .* abs(speed0);
    
    % Linearized dissipative forces (for controller design)
    % d_linear = -tau_r - 2*tau_d*|v0| (linearization)
    dv_lin = -tau_r - 2 * tau_d .* abs(speed0);
    % Linearized dissipative forces (for controller design)
    % d_linear = -tau_r - 2*tau_d*|v0| (linearization)
    dv_lin = -tau_r - 2 * tau_d .* abs(speed0);
    
    %% Gain Computation
    % Different control strategies for translational vs rotational DOFs
    for l = 1:dim_i
        if l <= 3
            % PID control for translational DOFs (surge, sway, heave)
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv_lin(l);
            Ti(l) = kp(l)/ki(l);
            Td(l) = kd(l)/kp(l);
            kt(l) = 1 / sqrt(Ti(l)*Td(l));  % Anti-windup gain
        else
            % PID control for rotational DOFs (roll, pitch, yaw)
            % Higher gains for better angular tracking
            kp(l) = mv(l)*((wn_angle^2) + 2*damp_angle*p_angle*wn_angle);
            ki(l) = p_angle*(wn_angle^2)*mv(l);
            kd(l) = (p_angle + 2*damp_angle*wn_angle)*mv(l) - dv_lin(l);
            Ti(l) = kp(l)/ki(l);
            Td(l) = kd(l)/kp(l);
            kt(l) = 1 / sqrt(Ti(l)*Td(l));  % Anti-windup gain
        end
    end
end