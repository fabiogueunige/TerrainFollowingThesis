%% KINEMATIC_MODEL - 6-DOF kinematic equations for underwater vehicles

function [eta, eta_dot] = kinematic_model(eta, nu, eta_dot_old, wRr, Ts)
    printDebug('       Kinematic Model\n');
    
    % Orientation (Euler angles)
    phi = eta(4);      % Roll
    theta = eta(5);    % Pitch
    psi = eta(6);      % Yaw
    
    % Body-fixed linear velocities
    u = nu(1);         % Surge (forward)
    v = nu(2);         % Sway (lateral)
    w = nu(3);         % Heave (vertical)
    
    % Body-fixed angular velocities
    p = nu(4);         % Roll rate
    q = nu(5);         % Pitch rate
    r = nu(6);         % Yaw rate
    
    %% Linear velocity transformation (Eq. 2.1)
    % Transform body-fixed linear velocities to inertial frame
    % η̇₁ = ᴵᴮR(η₂) * ν₁
    
    % Option 1: Use provided rotation matrix
    linear_vel_inertial = wRr * [u; v; w];
    
    % Option 2: Compute rotation matrix from Euler angles (if wRr not provided)
    % wRr_computed = rotz(psi) * roty(theta) * rotx(phi);
    % linear_vel_inertial = wRr_computed * [u; v; w];
    
    x_dot = linear_vel_inertial(1);
    y_dot = linear_vel_inertial(2);
    z_dot = linear_vel_inertial(3);
    
    %% Angular velocity transformation (Eq. 2.2)
    % Transform body-fixed angular rates to Euler angle rates
    % η̇₂ = T(η₂) * ν₂
    %
    % T(η₂) = [1   sin(φ)tan(θ)    cos(φ)tan(θ) ]
    %         [0      cos(φ)          -sin(φ)    ]
    %         [0   sin(φ)/cos(θ)   cos(φ)/cos(θ)]
    
    % Precompute trigonometric functions
    s_phi = sin(phi);
    c_phi = cos(phi);
    c_theta = cos(theta);
    t_theta = tan(theta);  % tan(θ) = sin(θ)/cos(θ)
    
    % Check for singularity (gimbal lock at θ = ±π/2)
    if abs(c_theta) < 1e-6
        warning('Kinematic model: Near gimbal lock (theta ≈ ±π/2). T matrix singular!');
        % Use small regularization to prevent division by zero
        c_theta = sign(c_theta) * max(abs(c_theta), 1e-6);
    end
    
    % Construct transformation matrix T(η₂)
    T = [1,   s_phi * t_theta,   c_phi * t_theta;
         0,        c_phi,              -s_phi;
         0,   s_phi / c_theta,   c_phi / c_theta];
    
    % Transform angular rates
    angular_rates = T * [p; q; r];
    
    phi_dot = angular_rates(1);
    theta_dot = angular_rates(2);
    psi_dot = angular_rates(3);
    
    %% Output combined derivative vector
    eta_dot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot];

    for ii = 1:6
        eta(ii) = integrator(eta(ii), eta_dot(ii), eta_dot_old(ii), Ts);
    end
    
    %% Debug output
    printDebug('Position rates: ẋ=%.3f, ẏ=%.3f, ż=%.3f [m/s]\n', ...
               x_dot, y_dot, z_dot);
    printDebug('Angle rates: φ̇=%.4f, θ̇=%.4f, ψ̇=%.4f [rad/s]\n', ...
               phi_dot, theta_dot, psi_dot);
    printDebug('Angle rates: φ̇=%.2f°/s, θ̇=%.2f°/s, ψ̇=%.2f°/s\n', ...
               rad2deg(phi_dot), rad2deg(theta_dot), rad2deg(psi_dot));
end
