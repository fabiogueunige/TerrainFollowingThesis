function [kp, ki, kd] = gainComputation(speed0, dim_i)
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    
    %% Control Design Parameters
    % Natural frequency and damping for pole placement
    % =========================================================
    % TUNING NOTES:
    % For terrain following, we need well-damped response
    % to prevent oscillations and overshoot near seafloor
    % =========================================================
    wn = 0.4;       % Natural frequency [rad/s]
    damp = 0.85;    % Increased damping (was 0.6, now 0.85 for less overshoot)
    p = 8;          % Additional pole (reduced from 10 for smoother response)
    
    % Initialize gain vectors
    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);

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

    % Linearized dissipative forces (for controller design)
    % d_linear = -tau_r - 2*tau_d*|v0| (linearization)
    dv_lin = -tau_r - 2 * tau_d .* abs(speed0);
    
    %% Gain Computation
    % Different control strategies for translational vs rotational DOFs
    for l = 1:dim_i
        if l == 1 || l == 2
            % PI control for surge and sway (horizontal motion)
            % No derivative action needed for stable horizontal dynamics
            kp(l) = 2*damp*wn*mv(l) - dv_lin(l);
            ki(l) = wn^2 * mv(l);
            kd(l) = 0;
        else
            % PID control for heave, roll, pitch, yaw
            % Derivative action helps with stability and faster response
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv_lin(l);
        end
    end
end