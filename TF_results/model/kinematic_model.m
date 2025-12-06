%% KINEMATIC_MODEL - 6-DOF kinematic equations for underwater vehicles
%
% Implements Fossen's kinematic equations that transform body-fixed velocities
% to inertial frame positions and Euler angle rates. These equations describe
% how the vehicle's motion in its own reference frame translates to motion
% in the world frame.
%
% SYNTAX:
%   [eta_dot] = kinematic_model(eta, nu, wRr)
%
% INPUTS:
%   eta  - Position and orientation vector [6x1]: [x, y, z, phi, theta, psi]'
%          x, y, z: Position in inertial frame [m]
%          phi: Roll angle [rad]
%          theta: Pitch angle [rad]
%          psi: Yaw angle [rad]
%   nu   - Body-fixed velocity vector [6x1]: [u, v, w, p, q, r]'
%          u: Surge velocity (forward) [m/s]
%          v: Sway velocity (lateral) [m/s]
%          w: Heave velocity (vertical) [m/s]
%          p: Roll rate [rad/s]
%          q: Pitch rate [rad/s]
%          r: Yaw rate [rad/s]
%   wRr  - Rotation matrix from body to world frame [3x3]
%          Alternative to computing from Euler angles
%
% OUTPUTS:
%   eta_dot - Time derivative of position/orientation [6x1]
%             [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]'
%
% EQUATIONS (Fossen, 2011):
%
%   Linear velocities (Eq. 2.1):
%   ┌───┐   ┌─────────────┐ ┌──┐
%   │η̇₁│ = │ᴵᴮR(η₂)   0₃ₓ₃│ │ν₁│  ⟺  η̇₁ = ᴵᴮR(η₂)ν₁
%   └───┘   └─────────────┘ └──┘
%
%   Where: η̇₁ = [ẋ, ẏ, ż]' = inertial frame linear velocities
%          ν₁ = [u, v, w]' = body-fixed linear velocities
%          ᴵᴮR(η₂) = rotation matrix from body to inertial frame
%
%   Angular velocities (Eq. 2.2):
%   ┌───┐   ┌─────────────────────────────────────┐ ┌──┐
%   │η̇₂│ = │1   sin(φ)tan(θ)    cos(φ)tan(θ) │ │ν₂│  ⟺  η̇₂ = T(η₂)ν₂
%   │   │   │0      cos(φ)          -sin(φ)    │ │  │
%   │   │   │0   sin(φ)/cos(θ)   cos(φ)/cos(θ)│ │  │
%   └───┘   └─────────────────────────────────────┘ └──┘
%
%   Where: η̇₂ = [φ̇, θ̇, ψ̇]' = Euler angle rates
%          ν₂ = [p, q, r]' = body-fixed angular velocities
%          T(η₂) = transformation matrix (depends on current orientation)
%
% SINGULARITIES:
%   T(η₂) becomes singular when θ = ±π/2 (gimbal lock)
%   In this case, cos(θ) = 0 and divisions by cos(θ) explode
%   This is a fundamental limitation of Euler angle representations
%
% PHYSICAL INTERPRETATION:
%   - Linear part: Velocities measured in body frame (u,v,w) are rotated
%     to get inertial frame velocities (ẋ,ẏ,ż) using rotation matrix
%   - Angular part: Body angular rates (p,q,r) are transformed to Euler
%     angle rates (φ̇,θ̇,ψ̇) using transformation T that accounts for
%     the current orientation
%
% NOTES:
%   - This is a purely geometric relationship (no dynamics/forces)
%   - The rotation matrix wRr can be precomputed or passed as input
%   - For numerical stability, avoid operating near θ = ±π/2
%   - Integration of η̇ gives position and orientation over time
%
% USAGE EXAMPLE:
%   % Current state
%   eta = [10; 5; -20; 0.1; -0.05; 1.2];  % pos + angles
%   nu = [0.5; 0.1; 0; 0.01; 0.02; 0.05];  % body velocities
%   wRr = rotz(psi)*roty(theta)*rotx(phi); % rotation matrix
%   
%   % Compute rates
%   eta_dot = kinematic_model(eta, nu, wRr);
%   
%   % Integrate (Euler method)
%   eta_new = eta + eta_dot * dt;
%
% See also: dynamic_model, rotz, roty, rotx
%
% REFERENCE:
%   Fossen, T. I. (2011). Handbook of Marine Craft Hydrodynamics 
%   and Motion Control. John Wiley & Sons.

function [eta_dot] = kinematic_model(eta, nu, wRr)
    printDebug('       Kinematic Model\n');
    
    %% Extract state components
    % Position (not needed for derivative computation)
    % x = eta(1);
    % y = eta(2);
    % z = eta(3);
    
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
    s_theta = sin(theta);
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
    
    %% Debug output
    printDebug('Position rates: ẋ=%.3f, ẏ=%.3f, ż=%.3f [m/s]\n', ...
               x_dot, y_dot, z_dot);
    printDebug('Angle rates: φ̇=%.4f, θ̇=%.4f, ψ̇=%.4f [rad/s]\n', ...
               phi_dot, theta_dot, psi_dot);
    printDebug('Angle rates: φ̇=%.2f°/s, θ̇=%.2f°/s, ψ̇=%.2f°/s\n', ...
               rad2deg(phi_dot), rad2deg(theta_dot), rad2deg(psi_dot));
end
