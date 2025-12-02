%% ============================================================================
%  SBES ANGLE OPTIMIZATION FOR 6-DOF AUV TERRAIN-FOLLOWING
%  ============================================================================
%  Multi-objective optimization of SBES sensor angle γ
%  System: 4 SBES sensors in X-configuration (N, S, E, W)
%  Objective: Maximize J(γ) = w1*f_res + w2*f_cov + w3*f_rob + w4*f_unc
%  ============================================================================

clear all; close all; clc;

%% ============================================================================
% 1. SYSTEM PARAMETERS (Physical characteristics)
%% ============================================================================

% ALTITUDE AND VELOCITY
h = 3.0;                % [m] Perpendicular distance from terrain (constant)
v = 0.3;                % [m/s] Surge velocity (forward speed)
t_react = 1;          % [s] Controller reaction time

% TERRAIN CHARACTERISTICS (3D - both axes)
alpha_terrain_max = deg2rad(20);  % [rad] Maximum terrain slope (roll direction)
beta_terrain_max = deg2rad(50);   % [rad] Maximum terrain slope (pitch direction)

% ROBOT ORIENTATION (3D - both roll and pitch)
% Different scenarios for gamma_max calculation
robot_roll = deg2rad(10);         % [rad] Robot roll angle (φ) - positive = right wing down
robot_pitch = deg2rad(5);        % [rad] Robot pitch angle (θ) - positive = nose up
tracking_error = deg2rad(30);     % [rad] Tracking error when following terrain

fprintf('System Parameters:\n');
fprintf('  - Altitude h = %.2f m\n', h);
fprintf('  - Velocity v = %.3f m/s\n', v);
fprintf('  - Reaction time = %.3f s\n', t_react);
fprintf('  - Max terrain slope (α_roll) = %.0f°\n', rad2deg(alpha_terrain_max));
fprintf('  - Max terrain slope (β_pitch) = %.0f°\n', rad2deg(beta_terrain_max));
fprintf('  - Robot roll (φ) = %.1f°\n', rad2deg(robot_roll));
fprintf('  - Robot pitch (θ) = %.1f°\n', rad2deg(robot_pitch));
fprintf('  - Tracking error (δ) = %.0f°\n\n', rad2deg(tracking_error));

%% ============================================================================
% 2. SENSOR SPECIFICATIONS (SBES 400 kHz - Adjustable)
%% ============================================================================

% SBES SENSOR CHARACTERISTICS
freq_sbes = 400;                % [kHz] SBES frequency [200]
sigma_sbes = 0.005;             % [m] Sensor resolution (depth measurement error) [0.01]
beamwidth = deg2rad(4.0);       % [deg] Beam opening angle [8.0]
f_ping = 20;                    % [Hz] Ping rate (measurement update frequency)

fprintf('Sensor Specifications (SBES %d kHz):\n', freq_sbes);
fprintf('  - Resolution σ = %.2f mm\n', sigma_sbes*1000);
fprintf('  - Beamwidth = %.1f°\n', rad2deg(beamwidth));
fprintf('  - Ping rate = %.0f Hz\n\n', f_ping);

%% ============================================================================
% 3. RESOLUTION CONSTRAINT (γ_min from SNR requirement)
%% ============================================================================

% SNR TARGET FOR ANGLE ESTIMATION
k_res = 100;            % [-] Signal-to-noise ratio desired for slope estimation
                        % Higher k_res → larger baseline required → larger γ_min
                        % [50 - 150]

% DERIVED: Minimum angle to achieve required baseline
% From: s ≥ k_res·σ_SBES → 2h·tan(γ) ≥ k_res·σ → γ ≥ arctan(...)
gamma_min = atan(k_res * sigma_sbes / (2*h));

s_min = k_res * sigma_sbes;  % Minimum baseline required

fprintf('Resolution Constraint (γ_min):\n');
fprintf('  - SNR target k_res = %d:1\n', k_res);
fprintf('  - Minimum baseline s_min = %.3f m\n', s_min);
fprintf('  - γ_min = %.2f°\n\n', rad2deg(gamma_min));

%% ============================================================================
% 4. TERRAIN CAPACITY CONSTRAINT (γ_max from geometry)
%% ============================================================================

% GEOMETRIC CONSTRAINT: Ray must hit terrain (angle < 90°)
% Effective angle: θ_eff = γ + |θ_robot - β_terrain| + beamwidth/2
% Constraint: θ_eff < 90° → γ < 90° - |θ_robot - β_terrain| - beamwidth/2

% SCENARIO 1: Perfect terrain-following
% Robot perfectly aligned with terrain: θ_robot = β_terrain
% θ_eff = γ + 0 + beamwidth/2 < 90°
theta_eff_perfect = 0 + beamwidth/2;
gamma_max_perfect = pi/2 - beamwidth/2;

% SCENARIO 2: Terrain-following with tracking error
% Robot tracks terrain but with error δ
% θ_eff = γ + δ + beamwidth/2 < 90°
theta_eff_tracking = tracking_error + beamwidth/2;
gamma_max_tracking = pi/2 - tracking_error - beamwidth/2;

% SCENARIO 3: Fixed horizontal robot on inclined terrain
% Robot stays horizontal (φ = 0, θ = 0), terrain at maximum slope
% Take the worst case between roll and pitch axes
theta_eff_fixed_roll = alpha_terrain_max + beamwidth/2;
theta_eff_fixed_pitch = beta_terrain_max + beamwidth/2;
theta_eff_fixed = max(theta_eff_fixed_roll, theta_eff_fixed_pitch);
gamma_max_fixed = pi/2 - max(alpha_terrain_max, beta_terrain_max) - beamwidth/2;

% SCENARIO 4: Current robot orientation on terrain (3D)
% Geometric correction for both axes independently
% ROLL axis (sensors E/W): influenced by φ_robot and α_terrain
if sign(robot_roll) == sign(alpha_terrain_max)
    effective_slope_roll = abs(abs(robot_roll) - abs(alpha_terrain_max)) + tracking_error;
else
    effective_slope_roll = abs(robot_roll) + abs(alpha_terrain_max) + tracking_error;
end

% PITCH axis (sensors N/S): influenced by θ_robot and β_terrain
if sign(robot_pitch) == sign(beta_terrain_max)
    effective_slope_pitch = abs(abs(robot_pitch) - abs(beta_terrain_max)) + tracking_error;
else
    effective_slope_pitch = abs(robot_pitch) + abs(beta_terrain_max) + tracking_error;
end

% Take worst case (most restrictive) between the two axes
effective_slope_current = max(effective_slope_roll, effective_slope_pitch);
theta_eff_current = effective_slope_current + beamwidth/2;
gamma_max_current = pi/2 - effective_slope_current - beamwidth/2;

% CHOOSE CONSTRAINT APPROACH
SCENARIO = 2;  % 1=Perfect, 2=Tracking, 3=Fixed horizontal, 4=Current orientation

switch SCENARIO
    case 1
        gamma_max = gamma_max_perfect;
        constraint_type = 'Perfect terrain-following';
        theta_eff_design = theta_eff_perfect;
    case 2
        gamma_max = gamma_max_tracking;
        constraint_type = sprintf('Tracking with δ=%.0f° error', rad2deg(tracking_error));
        theta_eff_design = theta_eff_tracking;
    case 3
        gamma_max = gamma_max_fixed;
        constraint_type = sprintf('Fixed horizontal (β_max=%.0f°)', rad2deg(beta_terrain_max));
        theta_eff_design = theta_eff_fixed;
    case 4
        gamma_max = gamma_max_current;
        constraint_type = sprintf('Current (θ=%.1f°, β=%.0f°, δ=%.0f°)', ...
                                 rad2deg(robot_pitch), rad2deg(beta_terrain_max), rad2deg(tracking_error));
        theta_eff_design = theta_eff_current;
end

% Ensure gamma_max is positive
if gamma_max < 0
    warning('gamma_max is negative (%.2f°)! Constraint too restrictive.', rad2deg(gamma_max));
    fprintf('Consider: 1) Reducing β_terrain_max, 2) Improving tracking, 3) Reducing θ_robot\n');
    gamma_max = deg2rad(45);  % Fallback value
end

fprintf('Terrain Capacity Constraint (γ_max):\n');
fprintf('  - Selected scenario: %s\n', constraint_type);
fprintf('  - Effective ray-terrain angle: θ_eff = %.2f°\n', rad2deg(theta_eff_design));
fprintf('  ┌────────────────────────────────────────────────────────────┐\n');
fprintf('  │ SCENARIO 1 (Perfect):   γ_max = %5.2f° (θ_eff=%4.1f°) │\n', ...
        rad2deg(gamma_max_perfect), rad2deg(theta_eff_perfect));
fprintf('  │ SCENARIO 2 (Tracking):  γ_max = %5.2f° (θ_eff=%4.1f°) │\n', ...
        rad2deg(gamma_max_tracking), rad2deg(theta_eff_tracking));
fprintf('  │ SCENARIO 3 (Fixed):     γ_max = %5.2f° (θ_eff=%4.1f°) │\n', ...
        rad2deg(gamma_max_fixed), rad2deg(theta_eff_fixed));
fprintf('  │ SCENARIO 4 (Current):   γ_max = %5.2f° (θ_eff=%4.1f°) │\n', ...
        rad2deg(gamma_max_current), rad2deg(theta_eff_current));
fprintf('  └────────────────────────────────────────────────────────────┘\n');
fprintf('  - SELECTED: Scenario %d → γ_max = %.2f°\n\n', SCENARIO, rad2deg(gamma_max));

%% ============================================================================
% 5. OBJECTIVE FUNCTION PARAMETERS (Multi-objective tuning)
%% ============================================================================

% COMPONENT WEIGHTS (must sum to 1.0)
w1 = 0.30;              % Weight for resolution quality (baseline size)
                        % Higher w1 → prefer larger γ → larger baseline
w2 = 0.20;              % Weight for coverage/look-ahead (anticipation)
                        % Higher w2 → prefer larger γ → better prediction
w3 = 0.30;              % Weight for acoustic robustness (return quality)
                        % Higher w3 → prefer γ ≈ 25° (empirical optimum)
w4 = 0.20;              % Weight for geometric stability (terrain handling)
                        % Higher w4 → prefer smaller γ → safer for steep terrain

weights = [w1, w2, w3, w4];
weight_sum = sum(weights);
if abs(weight_sum - 1.0) > 1e-6
    warning('Weights do not sum to 1.0! Sum = %.4f', weight_sum);
    weights = weights / weight_sum;  % Auto-normalize
    fprintf('Weights auto-normalized.\n\n');
end

fprintf('Objective Function Weights:\n');
fprintf('  - w1 (resolution)  = %.2f\n', w1);
fprintf('  - w2 (coverage)    = %.2f\n', w2);
fprintf('  - w3 (robustness)  = %.2f\n', w3);
fprintf('  - w4 (uncertainty) = %.2f\n', w4);
fprintf('  - Sum = %.2f ✓\n\n', sum(weights));

%% ============================================================================
% 6. COMPONENT TUNING PARAMETERS
%% ============================================================================

% RESOLUTION COMPONENT f1(γ) = tanh(2h·tan(γ)/s_ref)
s_ref = 1.0;            % [m] Reference baseline for normalization
                        % Larger s_ref → f1 increases slower with γ

% COVERAGE COMPONENT f2(γ) = 1 - exp(-k_c*(h·tan(γ) - v·t_react))
k_c = 5.0;              % [-] Scaling coefficient for coverage
                        % Higher k_c → sharper transition, more sensitive to small d_ahead

% ROBUSTNESS COMPONENT f3(γ) = exp(-(γ - γ_opt)²/(2σ_γ²))
gamma_opt_ref = deg2rad(25);    % [rad] Empirical optimal angle for acoustic return
sigma_gamma = deg2rad(15);      % [rad] Standard deviation of Gaussian preference
                                % Larger σ_γ → broader preference, less restrictive

% UNCERTAINTY COMPONENT f4(γ) = tanh(max(denom, 0.01)/0.3)
denom_min_safe = 0.01;  % [-] Minimum safe denominator (stability threshold)
                        % where denom = cos(γ) - sin(γ)·tan(θ_max)
scale_uncertainty = 0.3; % [-] Scaling factor for tanh function
                        % Larger scale → gentler penalty for instability

fprintf('Component Tuning Parameters:\n');
fprintf('  - s_ref (resolution scale) = %.2f m\n', s_ref);
fprintf('  - k_c (coverage scale) = %.2f\n', k_c);
fprintf('  - γ_opt (acoustic optimum) = %.0f°\n', rad2deg(gamma_opt_ref));
fprintf('  - σ_γ (acoustic bandwidth) = %.0f°\n', rad2deg(sigma_gamma));
fprintf('  - denom_min (stability threshold) = %.3f\n', denom_min_safe);
fprintf('  - scale_uncertainty (tanh scale) = %.2f\n\n', scale_uncertainty);

%% ============================================================================
% 7. OPTIMIZE γ USING FMINBND
%% ============================================================================

fprintf('Optimization Process:\n');
fprintf('  - Method: Bounded scalar minimization (FMINBND)\n');
fprintf('  - Search range: [%.2f°, %.2f°]\n\n', rad2deg(gamma_min), rad2deg(gamma_max));

% Objective function for fminbnd (minimizes, so we use negative J)
objective_neg = @(gamma) -objective_function(gamma, h, v, t_react, ...
                    alpha_terrain_max, beta_terrain_max, robot_roll, robot_pitch, tracking_error, ...
                    s_ref, k_c, gamma_opt_ref, sigma_gamma, ...
                    w1, w2, w3, w4, denom_min_safe, scale_uncertainty);

% Optimization with tolerance
options = optimset('TolX', 1e-8, 'Display', 'off');
[gamma_opt, J_neg_opt] = fminbnd(objective_neg, gamma_min, gamma_max, options);
J_opt = -J_neg_opt;

fprintf('OPTIMIZATION RESULT:\n');
fprintf('  ╔════════════════════════════════════════╗\n');
fprintf('  ║ Optimal angle: γ* = %.3f° = %.4f rad ║\n', rad2deg(gamma_opt), gamma_opt);
fprintf('  ║ Objective value: J(γ*) = %.6f     ║\n', J_opt);
fprintf('  ╚════════════════════════════════════════╝\n\n');

%% ============================================================================
% 8. COMPUTE METRICS AT OPTIMAL ANGLE
%% ============================================================================

% Re-compute components and metrics at optimum
baseline_opt = 2 * h * tan(gamma_opt);
d_ahead_opt = h * tan(gamma_opt);
r_range_opt = h / cos(gamma_opt);

% Ray-terrain geometry with robot orientation (3D)
% Compute effective slope for both axes

% ROLL axis (sensors E/W)
if sign(robot_roll) == sign(alpha_terrain_max)
    effective_slope_roll = abs(abs(robot_roll) - abs(alpha_terrain_max)) + tracking_error;
else
    effective_slope_roll = abs(robot_roll) + abs(alpha_terrain_max) + tracking_error;
end

% PITCH axis (sensors N/S)
if sign(robot_pitch) == sign(beta_terrain_max)
    effective_slope_pitch = abs(abs(robot_pitch) - abs(beta_terrain_max)) + tracking_error;
else
    effective_slope_pitch = abs(robot_pitch) + abs(beta_terrain_max) + tracking_error;
end

% Take worst case for stability
effective_slope = max(effective_slope_roll, effective_slope_pitch);
theta_ray_terrain_opt = gamma_opt + effective_slope;
denom_opt = cos(theta_ray_terrain_opt) / cos(effective_slope);
r_worst_case = h / max(abs(denom_opt), 0.001);  % Range in worst case

% Precision metrics
sigma_theta_opt = sigma_sbes / baseline_opt;  % [rad]
snr_angular = baseline_opt / sigma_sbes;

% Timing metrics
t_available = d_ahead_opt / v;  % [s]
margin_time_pct = (t_available - t_react) / t_react * 100;

% Sampling metrics
delta_s = v / f_ping;  % [m] Distance between pings
n_pings_per_baseline = baseline_opt / delta_s;

% Compute individual components
f_res_opt = tanh(baseline_opt / s_ref);
f_cov_opt = 1 - exp(-k_c * (d_ahead_opt - v*t_react));
f_rob_opt = exp(-(gamma_opt - gamma_opt_ref)^2 / (2*sigma_gamma^2));
f_unc_opt = tanh(max(denom_opt, denom_min_safe) / scale_uncertainty);

% Baseline correction for robot orientation (3D)
% Sensors N/S affected by pitch, sensors E/W affected by roll
baseline_correction_pitch = cos(robot_pitch);   % For N/S sensors
baseline_correction_roll = cos(robot_roll);     % For E/W sensors
baseline_correction_avg = (baseline_correction_pitch + baseline_correction_roll) / 2;
baseline_effective = baseline_opt * baseline_correction_avg;

% Precision degradation factor (average of both axes)
precision_degradation_pitch = 1 / baseline_correction_pitch;
precision_degradation_roll = 1 / baseline_correction_roll;
precision_degradation = (precision_degradation_pitch + precision_degradation_roll) / 2;

fprintf('OPERATIVE METRICS:\n\n');

fprintf('GEOMETRY:\n');
fprintf('  - Baseline (robot frame) = %.3f m\n', baseline_opt);
fprintf('  - Baseline (effective, 3D avg) = %.3f m\n', baseline_effective);
fprintf('  - Baseline correction (pitch axis, θ=%.1f°) = %.3f\n', rad2deg(robot_pitch), baseline_correction_pitch);
fprintf('  - Baseline correction (roll axis, φ=%.1f°) = %.3f\n', rad2deg(robot_roll), baseline_correction_roll);
fprintf('  - Baseline correction (average) = %.3f\n', baseline_correction_avg);
fprintf('  - Look-ahead distance = %.3f m\n', d_ahead_opt);
fprintf('  - SBES range (flat terrain, aligned robot) = %.3f m\n', r_range_opt);
fprintf('  - SBES range (worst case: 3D terrain+orientation) = %.2f m\n', r_worst_case);
fprintf('  - Footprint area ≈ %.2f m²\n\n', baseline_effective^2);

fprintf('RAY-TERRAIN GEOMETRY (3D):\n');
fprintf('  - Robot roll (φ) = %.1f°\n', rad2deg(robot_roll));
fprintf('  - Robot pitch (θ) = %.1f°\n', rad2deg(robot_pitch));
fprintf('  - Terrain slope (α_roll) = %.0f°\n', rad2deg(alpha_terrain_max));
fprintf('  - Terrain slope (β_pitch) = %.0f°\n', rad2deg(beta_terrain_max));
fprintf('  - Tracking error (δ) = %.0f°\n', rad2deg(tracking_error));
fprintf('  - Effective slope (roll axis) = %.1f°\n', rad2deg(effective_slope_roll));
fprintf('  - Effective slope (pitch axis) = %.1f°\n', rad2deg(effective_slope_pitch));
fprintf('  - Effective slope (worst case) = %.1f°\n', rad2deg(effective_slope));
fprintf('  - Ray-terrain angle = γ + effective_slope = %.2f°\n', rad2deg(theta_ray_terrain_opt));
fprintf('  - Margin to 90° = %.2f°\n\n', rad2deg(pi/2 - theta_ray_terrain_opt));

fprintf('PRECISION:\n');
fprintf('  - Angle estimation error σ_α = %.3f° = %.2f mrad\n', rad2deg(sigma_theta_opt), sigma_theta_opt*1000);
fprintf('  - Angular SNR = %.0f:1\n', snr_angular);
fprintf('  - Precision degradation (pitch axis) = %.2f×\n', precision_degradation_pitch);
fprintf('  - Precision degradation (roll axis) = %.2f×\n', precision_degradation_roll);
fprintf('  - Precision degradation (average) = %.2f×\n\n', precision_degradation);

fprintf('TIMING:\n');
fprintf('  - Time available for reaction = %.2f s\n', t_available);
fprintf('  - Time required = %.3f s\n', t_react);
fprintf('  - Safety margin = %.2f s = %.0f%%\n\n', t_available - t_react, margin_time_pct);

fprintf('SAMPLING:\n');
fprintf('  - Distance between pings = %.4f m\n', delta_s);
fprintf('  - Pings per baseline = %.0f\n\n', n_pings_per_baseline);

fprintf('STABILITY:\n');
fprintf('  - Ray-terrain cosine ratio = %.4f\n', denom_opt);
fprintf('  - Critical angle (ray ∥ terrain) at γ = %.2f°\n', rad2deg(pi/2 - effective_slope));
if denom_opt > 0.1
    fprintf('  - Status: ✓ STABLE (safe margin, all rays hit terrain)\n\n');
elseif denom_opt > 0
    fprintf('  - Status: ⚠ MARGINAL (small margin, verify in practice)\n\n');
else
    fprintf('  - Status: ✗ UNSTABLE (rays may diverge, reduce γ or improve tracking)\n\n');
end

fprintf('OBJECTIVE COMPONENTS:\n');
fprintf('  - f_resolution = %.4f  →  contribution = %.4f (w1×f1)\n', f_res_opt, w1*f_res_opt);
fprintf('  - f_coverage = %.4f    →  contribution = %.4f (w2×f2)\n', f_cov_opt, w2*f_cov_opt);
fprintf('  - f_robustness = %.4f  →  contribution = %.4f (w3×f3)\n', f_rob_opt, w3*f_rob_opt);
fprintf('  - f_uncertainty = %.4f →  contribution = %.4f (w4×f4)\n\n', f_unc_opt, w4*f_unc_opt);

%% ============================================================================
% 9. GENERATE SENSITIVITY ANALYSIS PLOT
%% ============================================================================

% Evaluate J over the entire feasible range
gamma_plot = linspace(gamma_min, gamma_max, 200);
J_plot = zeros(size(gamma_plot));
f1_plot = zeros(size(gamma_plot));
f2_plot = zeros(size(gamma_plot));
f3_plot = zeros(size(gamma_plot));
f4_plot = zeros(size(gamma_plot));

for i = 1:length(gamma_plot)
    J_plot(i) = -objective_neg(gamma_plot(i));
    
    % Component 1: Resolution with 3D correction (average of both axes)
    baseline = 2 * h * tan(gamma_plot(i));
    baseline_eff_pitch_i = baseline * cos(robot_pitch);
    baseline_eff_roll_i = baseline * cos(robot_roll);
    baseline_eff_i = (baseline_eff_pitch_i + baseline_eff_roll_i) / 2;
    f1_plot(i) = tanh(baseline_eff_i / s_ref);
    
    % Component 2: Coverage
    d_ahead = h * tan(gamma_plot(i));
    f2_plot(i) = 1 - exp(-k_c * (d_ahead - v*t_react));
    
    % Component 3: Robustness
    f3_plot(i) = exp(-(gamma_plot(i) - gamma_opt_ref)^2 / (2*sigma_gamma^2));
    
    % Component 4: Uncertainty with 3D geometry (worst case)
    % Roll axis
    if sign(robot_roll) == sign(alpha_terrain_max)
        eff_slope_roll_i = abs(abs(robot_roll) - abs(alpha_terrain_max)) + tracking_error;
    else
        eff_slope_roll_i = abs(robot_roll) + abs(alpha_terrain_max) + tracking_error;
    end
    % Pitch axis
    if sign(robot_pitch) == sign(beta_terrain_max)
        eff_slope_pitch_i = abs(abs(robot_pitch) - abs(beta_terrain_max)) + tracking_error;
    else
        eff_slope_pitch_i = abs(robot_pitch) + abs(beta_terrain_max) + tracking_error;
    end
    eff_slope_i = max(eff_slope_roll_i, eff_slope_pitch_i);
    theta_eff_i = gamma_plot(i) + eff_slope_i;
    denom_plot = cos(theta_eff_i) / cos(eff_slope_i);
    f4_plot(i) = tanh(max(denom_plot, denom_min_safe) / scale_uncertainty);
end

% Create figure with subplots
figure('Name', 'SBES Angle Optimization Analysis', 'NumberTitle', 'off', ...
       'Position', [100 100 1200 800]);

% Subplot 1: Overall objective function
subplot(2, 3, 1);
plot(gamma_plot*180/pi, J_plot, 'b-', 'LineWidth', 2);
hold on;
plot(rad2deg(gamma_opt), J_opt, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Angle γ [degrees]');
ylabel('Objective J(γ)');
title('Overall Objective Function');
grid on;
legend('J(γ)', sprintf('Optimum: γ*=%.2f°', rad2deg(gamma_opt)), 'Location', 'best');

% Subplot 2-5: Individual components
subplot(2, 3, 2);
plot(rad2deg(gamma_plot), f1_plot, 'LineWidth', 2);
hold on;
plot(rad2deg(gamma_opt), f_res_opt, 'r*', 'MarkerSize', 12);
xlabel('Angle γ [degrees]');
ylabel('f_resolution');
title(sprintf('Component 1: Resolution (w₁=%.2f)', w1));
grid on;

subplot(2, 3, 3);
plot(rad2deg(gamma_plot), f2_plot, 'LineWidth', 2);
hold on;
plot(rad2deg(gamma_opt), f_cov_opt, 'r*', 'MarkerSize', 12);
xlabel('Angle γ [degrees]');
ylabel('f_coverage');
title(sprintf('Component 2: Coverage (w₂=%.2f)', w2));
grid on;

subplot(2, 3, 4);
plot(rad2deg(gamma_plot), f3_plot, 'LineWidth', 2);
hold on;
plot(rad2deg(gamma_opt), f_rob_opt, 'r*', 'MarkerSize', 12);
xlabel('Angle γ [degrees]');
ylabel('f_robustness');
title(sprintf('Component 3: Robustness (w₃=%.2f)', w3));
grid on;

subplot(2, 3, 5);
plot(rad2deg(gamma_plot), f4_plot, 'LineWidth', 2);
hold on;
plot(rad2deg(gamma_opt), f_unc_opt, 'r*', 'MarkerSize', 12);
xlabel('Angle γ [degrees]');
ylabel('f_uncertainty');
title(sprintf('Component 4: Uncertainty (w₄=%.2f)', w4));
grid on;

% Subplot 6: Weighted components
subplot(2, 3, 6);
plot(rad2deg(gamma_plot), w1*f1_plot, 'LineWidth', 1.5, 'DisplayName', sprintf('w₁·f₁ (%.2f)', w1));
hold on;
plot(rad2deg(gamma_plot), w2*f2_plot, 'LineWidth', 1.5, 'DisplayName', sprintf('w₂·f₂ (%.2f)', w2));
plot(rad2deg(gamma_plot), w3*f3_plot, 'LineWidth', 1.5, 'DisplayName', sprintf('w₃·f₃ (%.2f)', w3));
plot(rad2deg(gamma_plot), w4*f4_plot, 'LineWidth', 1.5, 'DisplayName', sprintf('w₄·f₄ (%.2f)', w4));
plot(rad2deg(gamma_opt), J_opt, 'r*', 'MarkerSize', 15, 'DisplayName', 'Optimum');
ylabel('Weighted Components');
title('Weighted Contributions');
legend('Location', 'best');
grid on;

sgtitle('SBES Angle Optimization: Multi-Objective Analysis', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure
% saveas(gcf, 'sbes_sensitivity_analysis.png');
% fprintf('✓ Figure saved: sbes_sensitivity_analysis.png\n');

%% ============================================================================
% 10. GEOMETRIC CONSTRAINT VISUALIZATION
%% ============================================================================

figure('Name', 'SBES Geometric Constraints', 'NumberTitle', 'off', ...
       'Position', [150 150 800 600]);

% Create angle range for visualization
gamma_viz = linspace(0, pi/2, 100);

% Compute feasibility for different scenarios
feasible_perfect = gamma_viz <= gamma_max_perfect;
feasible_tracking = gamma_viz <= gamma_max_tracking;
feasible_fixed = gamma_viz <= gamma_max_fixed;

% Plot feasibility regions
subplot(2,1,1);
hold on;
area(rad2deg(gamma_viz), feasible_perfect, 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.3, ...
     'DisplayName', 'Perfect (terrain-following)');
area(rad2deg(gamma_viz), feasible_tracking, 'FaceColor', [1 1 0.7], 'FaceAlpha', 0.5, ...
     'DisplayName', sprintf('Tracking (δ=%.0f° error)', rad2deg(tracking_error)));
area(rad2deg(gamma_viz), feasible_fixed, 'FaceColor', [1 0.7 0.7], 'FaceAlpha', 0.5, ...
     'DisplayName', sprintf('Fixed horizontal (β=%.0f°)', rad2deg(beta_terrain_max)));

% Mark constraint boundaries
plot([rad2deg(gamma_min) rad2deg(gamma_min)], [0 1], 'b--', 'LineWidth', 2, ...
     'DisplayName', sprintf('γ_min = %.2f° (resolution)', rad2deg(gamma_min)));
plot([rad2deg(gamma_max_perfect) rad2deg(gamma_max_perfect)], [0 1], 'g--', 'LineWidth', 2, ...
     'DisplayName', sprintf('γ_max (perfect) = %.2f°', rad2deg(gamma_max_perfect)));
plot([rad2deg(gamma_max_fixed) rad2deg(gamma_max_fixed)], [0 1], 'r--', 'LineWidth', 2, ...
     'DisplayName', sprintf('γ_max (fixed) = %.2f°', rad2deg(gamma_max_fixed)));

% Mark optimal angle
plot([rad2deg(gamma_opt) rad2deg(gamma_opt)], [0 1], 'k-', 'LineWidth', 3, ...
     'DisplayName', sprintf('γ* = %.2f° (optimal)', rad2deg(gamma_opt)));

xlabel('SBES Angle γ [degrees]');
ylabel('Feasibility');
title('Geometric Constraints and Feasibility Regions');
legend('Location', 'best');
grid on;
xlim([0 90]);
ylim([0 1.2]);

% Subplot 2: Effective ray angle vs terrain slope
subplot(2,1,2);
beta_range = linspace(0, beta_terrain_max, 50);
gamma_test = [gamma_min, gamma_opt, gamma_max_perfect];
gamma_labels = {sprintf('γ_min=%.1f°', rad2deg(gamma_min)), ...
                sprintf('γ*=%.1f°', rad2deg(gamma_opt)), ...
                sprintf('γ_max=%.1f°', rad2deg(gamma_max_perfect))};
colors = {'b', 'k', 'r'};

hold on;
for i = 1:length(gamma_test)
    % Effective angle = angle between ray and terrain normal
    % When robot is horizontal: θ_eff = γ + β
    theta_eff_fixed = gamma_test(i) + beta_range;
    
    % When robot adapts to terrain: θ_eff = γ (constant)
    theta_eff_adapt = gamma_test(i) * ones(size(beta_range));
    
    plot(rad2deg(beta_range), rad2deg(theta_eff_fixed), [colors{i} '--'], ...
         'LineWidth', 1.5, 'DisplayName', [gamma_labels{i} ' (fixed)']);
    plot(rad2deg(beta_range), rad2deg(theta_eff_adapt), [colors{i} '-'], ...
         'LineWidth', 2, 'DisplayName', [gamma_labels{i} ' (adapted)']);
end

% Mark 90° limit
plot([0 rad2deg(beta_terrain_max)], [90 90], 'r:', 'LineWidth', 2, 'DisplayName', '90° limit');

xlabel('Terrain Slope β [degrees]');
ylabel('Effective Ray Angle θ_{eff} [degrees]');
title('Ray-Terrain Angle vs Terrain Slope');
legend('Location', 'northwest');
grid on;
xlim([0 rad2deg(beta_terrain_max)]);
ylim([0 100]);

sgtitle('SBES Geometric Constraints Analysis', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure
% saveas(gcf, 'sbes_geometric_constraints.png');
% fprintf('✓ Figure saved: sbes_geometric_constraints.png\n');

%% ============================================================================
% 11. PITCH SENSITIVITY ANALYSIS
%% ============================================================================

figure('Name', 'Pitch Impact Analysis', 'NumberTitle', 'off', ...
       'Position', [200 100 1000 700]);

% Test range for robot pitch
pitch_range = linspace(-deg2rad(30), deg2rad(30), 50);

% Subplot 1: gamma_max vs pitch
subplot(2,2,1);
gamma_max_vs_pitch = zeros(size(pitch_range));
for i = 1:length(pitch_range)
    eff_slope_i = abs(pitch_range(i)) + beta_terrain_max + tracking_error;
    gamma_max_vs_pitch(i) = pi/2 - eff_slope_i - beamwidth/2;
end

plot(rad2deg(pitch_range), rad2deg(gamma_max_vs_pitch), 'b-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_pitch), rad2deg(gamma_max), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot([rad2deg(pitch_range(1)), rad2deg(pitch_range(end))], [0 0], 'k--', 'LineWidth', 1);
xlabel('Robot Pitch θ [degrees]');
ylabel('γ_{max} [degrees]');
title('Maximum Feasible SBES Angle vs Robot Pitch');
grid on;
legend('γ_{max}(θ)', 'Current', 'Zero line', 'Location', 'best');

% Subplot 2: Effective baseline vs pitch
subplot(2,2,2);
baseline_vs_pitch = 2 * h * tan(gamma_opt) .* cos(abs(pitch_range));

plot(rad2deg(pitch_range), baseline_vs_pitch, 'r-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_pitch), baseline_effective, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Robot Pitch θ [degrees]');
ylabel('Effective Baseline [m]');
title('Effective Baseline vs Robot Pitch');
grid on;
legend('Baseline_{eff}(θ)', 'Current', 'Location', 'best');

% Subplot 3: Ray-terrain angle vs pitch
subplot(2,2,3);
theta_ray_vs_pitch = zeros(size(pitch_range));
for i = 1:length(pitch_range)
    eff_slope_i = abs(pitch_range(i)) + beta_terrain_max + tracking_error;
    theta_ray_vs_pitch(i) = gamma_opt + eff_slope_i;
end

plot(rad2deg(pitch_range), rad2deg(theta_ray_vs_pitch), 'g-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_pitch), rad2deg(theta_ray_terrain_opt), 'g*', 'MarkerSize', 15, 'LineWidth', 2);
plot([rad2deg(pitch_range(1)), rad2deg(pitch_range(end))], [90 90], 'r--', 'LineWidth', 2);
xlabel('Robot Pitch θ [degrees]');
ylabel('Ray-Terrain Angle [degrees]');
title('Ray-Terrain Angle vs Robot Pitch');
grid on;
legend('θ_{ray}(θ)', 'Current', '90° limit', 'Location', 'best');
ylim([0 100]);

% Subplot 4: Combined metrics
subplot(2,2,4);
orient_angles = linspace(0, deg2rad(30), 50);
precision_deg = 1 ./ cos(orient_angles);
baseline_reduction = cos(orient_angles);

yyaxis left;
plot(rad2deg(orient_angles), precision_deg, 'b-', 'LineWidth', 2);
ylabel('Precision Degradation Factor', 'Color', 'b');
hold on;
plot(rad2deg(abs(robot_pitch)), precision_degradation, 'bo', 'MarkerSize', 10, 'LineWidth', 2);

yyaxis right;
plot(rad2deg(orient_angles), baseline_reduction * 100, 'r-', 'LineWidth', 2);
ylabel('Baseline Efficiency [%]', 'Color', 'r');
plot(rad2deg(abs(robot_pitch)), baseline_correction_avg * 100, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

xlabel('Robot Orientation |φ|, |θ| [degrees]');
title('Measurement Quality vs Robot Orientation (3D)');
grid on;
legend('Precision degradation', 'Current', 'Baseline efficiency', 'Current', 'Location', 'best');

sgtitle('SBES Performance vs Robot Pitch', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure
% saveas(gcf, 'sbes_pitch_impact.png');
% fprintf('✓ Figure saved: sbes_pitch_impact.png\n');

%% ============================================================================
% 12. ROLL vs PITCH SEPARATE ANALYSIS
%% ============================================================================

figure('Name', 'Roll vs Pitch Separate Analysis', 'NumberTitle', 'off', ...
       'Position', [250 50 1200 900]);

% Test ranges for roll and pitch independently
roll_range = linspace(-deg2rad(60), deg2rad(60), 50);
pitch_range = linspace(-deg2rad(60), deg2rad(60), 50);

% === ROLL AXIS ANALYSIS (keep pitch constant) ===

% Subplot 1: gamma_max vs roll (with constant pitch)
subplot(3,3,1);
gamma_max_vs_roll = zeros(size(roll_range));
for i = 1:length(roll_range)
    if sign(roll_range(i)) == sign(alpha_terrain_max)
        eff_slope_roll_i = abs(abs(roll_range(i)) - abs(alpha_terrain_max)) + tracking_error;
    else
        eff_slope_roll_i = abs(roll_range(i)) + abs(alpha_terrain_max) + tracking_error;
    end
    % Keep pitch constant at current value
    if sign(robot_pitch) == sign(beta_terrain_max)
        eff_slope_pitch_i = abs(abs(robot_pitch) - abs(beta_terrain_max)) + tracking_error;
    else
        eff_slope_pitch_i = abs(robot_pitch) + abs(beta_terrain_max) + tracking_error;
    end
    eff_slope_worst = max(eff_slope_roll_i, eff_slope_pitch_i);
    gamma_max_vs_roll(i) = pi/2 - eff_slope_worst - beamwidth/2;
end
plot(rad2deg(roll_range), rad2deg(gamma_max_vs_roll), 'b-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_roll), rad2deg(gamma_max), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot([rad2deg(roll_range(1)), rad2deg(roll_range(end))], [0 0], 'k--', 'LineWidth', 1);
xlabel('Robot Roll φ [degrees]');
ylabel('γ_{max} [degrees]');
title('Max Feasible Angle vs Roll');
grid on;
legend('γ_{max}(φ)', 'Current', 'Location', 'best');

% Subplot 2: Effective baseline vs roll
subplot(3,3,2);
baseline_vs_roll = 2 * h * tan(gamma_opt) .* cos(abs(roll_range));
plot(rad2deg(roll_range), baseline_vs_roll, 'b-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_roll), baseline_opt * cos(robot_roll), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Robot Roll φ [degrees]');
ylabel('Effective Baseline [m]');
title('Baseline (E/W sensors) vs Roll');
grid on;
legend('Baseline_{eff}(φ)', 'Current', 'Location', 'best');

% Subplot 3: Ray-terrain angle vs roll
subplot(3,3,3);
theta_ray_vs_roll = zeros(size(roll_range));
for i = 1:length(roll_range)
    if sign(roll_range(i)) == sign(alpha_terrain_max)
        eff_slope_roll_i = abs(abs(roll_range(i)) - abs(alpha_terrain_max)) + tracking_error;
    else
        eff_slope_roll_i = abs(roll_range(i)) + abs(alpha_terrain_max) + tracking_error;
    end
    theta_ray_vs_roll(i) = gamma_opt + eff_slope_roll_i;
end
plot(rad2deg(roll_range), rad2deg(theta_ray_vs_roll), 'b-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_roll), rad2deg(gamma_opt + effective_slope_roll), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot([rad2deg(roll_range(1)), rad2deg(roll_range(end))], [90 90], 'r--', 'LineWidth', 2);
xlabel('Robot Roll φ [degrees]');
ylabel('Ray-Terrain Angle [degrees]');
title('Ray-Terrain Angle (Roll Axis)');
grid on;
legend('θ_{ray}(φ)', 'Current', '90° limit', 'Location', 'best');
ylim([0 100]);

% === PITCH AXIS ANALYSIS (keep roll constant) ===

% Subplot 4: gamma_max vs pitch (with constant roll)
subplot(3,3,4);
gamma_max_vs_pitch = zeros(size(pitch_range));
for i = 1:length(pitch_range)
    % Keep roll constant at current value
    if sign(robot_roll) == sign(alpha_terrain_max)
        eff_slope_roll_i = abs(abs(robot_roll) - abs(alpha_terrain_max)) + tracking_error;
    else
        eff_slope_roll_i = abs(robot_roll) + abs(alpha_terrain_max) + tracking_error;
    end
    if sign(pitch_range(i)) == sign(beta_terrain_max)
        eff_slope_pitch_i = abs(abs(pitch_range(i)) - abs(beta_terrain_max)) + tracking_error;
    else
        eff_slope_pitch_i = abs(pitch_range(i)) + abs(beta_terrain_max) + tracking_error;
    end
    eff_slope_worst = max(eff_slope_roll_i, eff_slope_pitch_i);
    gamma_max_vs_pitch(i) = pi/2 - eff_slope_worst - beamwidth/2;
end
plot(rad2deg(pitch_range), rad2deg(gamma_max_vs_pitch), 'g-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_pitch), rad2deg(gamma_max), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot([rad2deg(pitch_range(1)), rad2deg(pitch_range(end))], [0 0], 'k--', 'LineWidth', 1);
xlabel('Robot Pitch θ [degrees]');
ylabel('γ_{max} [degrees]');
title('Max Feasible Angle vs Pitch');
grid on;
legend('γ_{max}(θ)', 'Current', 'Location', 'best');

% Subplot 5: Effective baseline vs pitch
subplot(3,3,5);
baseline_vs_pitch = 2 * h * tan(gamma_opt) .* cos(abs(pitch_range));
plot(rad2deg(pitch_range), baseline_vs_pitch, 'g-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_pitch), baseline_opt * cos(robot_pitch), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Robot Pitch θ [degrees]');
ylabel('Effective Baseline [m]');
title('Baseline (N/S sensors) vs Pitch');
grid on;
legend('Baseline_{eff}(θ)', 'Current', 'Location', 'best');

% Subplot 6: Ray-terrain angle vs pitch
subplot(3,3,6);
theta_ray_vs_pitch = zeros(size(pitch_range));
for i = 1:length(pitch_range)
    if sign(pitch_range(i)) == sign(beta_terrain_max)
        eff_slope_pitch_i = abs(abs(pitch_range(i)) - abs(beta_terrain_max)) + tracking_error;
    else
        eff_slope_pitch_i = abs(pitch_range(i)) + abs(beta_terrain_max) + tracking_error;
    end
    theta_ray_vs_pitch(i) = gamma_opt + eff_slope_pitch_i;
end
plot(rad2deg(pitch_range), rad2deg(theta_ray_vs_pitch), 'g-', 'LineWidth', 2);
hold on;
plot(rad2deg(robot_pitch), rad2deg(gamma_opt + effective_slope_pitch), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot([rad2deg(pitch_range(1)), rad2deg(pitch_range(end))], [90 90], 'r--', 'LineWidth', 2);
xlabel('Robot Pitch θ [degrees]');
ylabel('Ray-Terrain Angle [degrees]');
title('Ray-Terrain Angle (Pitch Axis)');
grid on;
legend('θ_{ray}(θ)', 'Current', '90° limit', 'Location', 'best');
ylim([0 100]);

% === COMPARISON PLOTS ===

% Subplot 7: Effective slopes comparison
subplot(3,3,7);
eff_slope_roll_array = zeros(size(roll_range));
for i = 1:length(roll_range)
    if sign(roll_range(i)) == sign(alpha_terrain_max)
        eff_slope_roll_array(i) = abs(abs(roll_range(i)) - abs(alpha_terrain_max)) + tracking_error;
    else
        eff_slope_roll_array(i) = abs(roll_range(i)) + abs(alpha_terrain_max) + tracking_error;
    end
end
eff_slope_pitch_array = zeros(size(pitch_range));
for i = 1:length(pitch_range)
    if sign(pitch_range(i)) == sign(beta_terrain_max)
        eff_slope_pitch_array(i) = abs(abs(pitch_range(i)) - abs(beta_terrain_max)) + tracking_error;
    else
        eff_slope_pitch_array(i) = abs(pitch_range(i)) + abs(beta_terrain_max) + tracking_error;
    end
end
plot(rad2deg(roll_range), rad2deg(eff_slope_roll_array), 'b-', 'LineWidth', 2, 'DisplayName', 'Roll axis');
hold on;
plot(rad2deg(pitch_range), rad2deg(eff_slope_pitch_array), 'g-', 'LineWidth', 2, 'DisplayName', 'Pitch axis');
plot(rad2deg(robot_roll), rad2deg(effective_slope_roll), 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Current roll');
plot(rad2deg(robot_pitch), rad2deg(effective_slope_pitch), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Current pitch');
xlabel('Robot Orientation [degrees]');
ylabel('Effective Slope [degrees]');
title('Effective Slope Comparison');
grid on;
legend('Location', 'best');

% Subplot 8: Precision degradation comparison
subplot(3,3,8);
precision_roll = 1 ./ cos(abs(roll_range));
precision_pitch = 1 ./ cos(abs(pitch_range));
plot(rad2deg(roll_range), precision_roll, 'b-', 'LineWidth', 2, 'DisplayName', 'Roll axis (E/W)');
hold on;
plot(rad2deg(pitch_range), precision_pitch, 'g-', 'LineWidth', 2, 'DisplayName', 'Pitch axis (N/S)');
plot(rad2deg(robot_roll), precision_degradation_roll, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Current roll');
plot(rad2deg(robot_pitch), precision_degradation_pitch, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Current pitch');
xlabel('Robot Orientation [degrees]');
ylabel('Precision Degradation Factor');
title('Precision Degradation: Roll vs Pitch');
grid on;
legend('Location', 'best');

% Subplot 9: 2D heatmap of worst-case effective slope
subplot(3,3,9);
[Roll_grid, Pitch_grid] = meshgrid(linspace(-60, 60, 30), linspace(-60, 60, 30));
Eff_slope_grid = zeros(size(Roll_grid));
for i = 1:size(Roll_grid, 1)
    for j = 1:size(Roll_grid, 2)
        roll_val = deg2rad(Roll_grid(i,j));
        pitch_val = deg2rad(Pitch_grid(i,j));
        % Roll contribution
        if sign(roll_val) == sign(alpha_terrain_max)
            eff_roll = abs(abs(roll_val) - abs(alpha_terrain_max)) + tracking_error;
        else
            eff_roll = abs(roll_val) + abs(alpha_terrain_max) + tracking_error;
        end
        % Pitch contribution
        if sign(pitch_val) == sign(beta_terrain_max)
            eff_pitch = abs(abs(pitch_val) - abs(beta_terrain_max)) + tracking_error;
        else
            eff_pitch = abs(pitch_val) + abs(beta_terrain_max) + tracking_error;
        end
        Eff_slope_grid(i,j) = rad2deg(max(eff_roll, eff_pitch));
    end
end
contourf(Roll_grid, Pitch_grid, Eff_slope_grid, 20);
hold on;
plot(rad2deg(robot_roll), rad2deg(robot_pitch), 'r*', 'MarkerSize', 20, 'LineWidth', 3);
colorbar;
xlabel('Robot Roll φ [degrees]');
ylabel('Robot Pitch θ [degrees]');
title('Worst-Case Effective Slope [°]');
grid on;

sgtitle('Roll vs Pitch: Independent Axis Analysis', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure
% saveas(gcf, 'sbes_roll_pitch_separate.png');
% fprintf('✓ Figure saved: sbes_roll_pitch_separate.png\n');

%% ============================================================================
% 13. OPTIMIZATION RESULTS FOR ALL SCENARIOS
%% ============================================================================

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════════\n');
fprintf('                    OPTIMIZATION SUMMARY FOR ALL SCENARIOS\n');
fprintf('═══════════════════════════════════════════════════════════════════\n\n');

% Store original scenario selection
original_scenario = SCENARIO;

% Arrays to store results
gamma_opt_scenarios = zeros(4, 1);
J_opt_scenarios = zeros(4, 1);
scenario_names = {'Perfect (θ=β)', 'Tracking (δ error)', 'Fixed (θ=0)', 'Current config'};

% Optimize for each scenario
for s = 1:4
    % Select gamma_max for this scenario
    switch s
        case 1
            gamma_max_s = gamma_max_perfect;
        case 2
            gamma_max_s = gamma_max_tracking;
        case 3
            gamma_max_s = gamma_max_fixed;
        case 4
            gamma_max_s = gamma_max_current;
    end
    
    % Skip if gamma_max is negative
    if gamma_max_s < 0
        gamma_opt_scenarios(s) = NaN;
        J_opt_scenarios(s) = NaN;
        continue;
    end
    
    % Optimize for this scenario
    [gamma_opt_s, J_neg_opt_s] = fminbnd(objective_neg, gamma_min, gamma_max_s, options);
    gamma_opt_scenarios(s) = gamma_opt_s;
    J_opt_scenarios(s) = -J_neg_opt_s;
end

% Display comparison table
fprintf('┌─────────┬─────────────────────────┬──────────────┬──────────────┬────────────┐\n');
fprintf('│ Scenario│ Description             │ γ_max [°]    │ γ* [°]       │ J(γ*)      │\n');
fprintf('├─────────┼─────────────────────────┼──────────────┼──────────────┼────────────┤\n');

for s = 1:4
    % Get gamma_max for this scenario
    switch s
        case 1
            gmax = gamma_max_perfect;
        case 2
            gmax = gamma_max_tracking;
        case 3
            gmax = gamma_max_fixed;
        case 4
            gmax = gamma_max_current;
    end
    
    if isnan(gamma_opt_scenarios(s))
        fprintf('│    %d    │ %-23s │ %7.2f      │     N/A      │    N/A     │\n', ...
                s, scenario_names{s}, rad2deg(gmax));
    else
        % Mark the selected scenario with ►
        marker = '   ';
        if s == original_scenario
            marker = ' ►';
        end
        fprintf('│    %d    │ %-23s │ %7.2f      │ %7.2f%s    │  %.6f  │\n', ...
                s, scenario_names{s}, rad2deg(gmax), rad2deg(gamma_opt_scenarios(s)), marker, J_opt_scenarios(s));
    end
end

fprintf('└─────────┴─────────────────────────┴──────────────┴──────────────┴────────────┘\n');
fprintf('► = Currently selected scenario (SCENARIO = %d)\n\n', original_scenario);

% Display key insights
fprintf('KEY INSIGHTS:\n');

% Find valid scenarios
valid_scenarios = ~isnan(gamma_opt_scenarios);
if sum(valid_scenarios) > 1
    [max_gamma, idx_max] = max(gamma_opt_scenarios(valid_scenarios));
    [min_gamma, idx_min] = min(gamma_opt_scenarios(valid_scenarios));
    valid_idx = find(valid_scenarios);
    
    fprintf('  • γ* range: %.2f° (Scenario %d) to %.2f° (Scenario %d)\n', ...
            rad2deg(min_gamma), valid_idx(idx_min), rad2deg(max_gamma), valid_idx(idx_max));
    fprintf('  • Spread: %.2f° difference between scenarios\n', rad2deg(max_gamma - min_gamma));
end

% Compare current scenario to others
if original_scenario > 1 && ~isnan(gamma_opt_scenarios(1))
    delta_from_perfect = gamma_opt_scenarios(original_scenario) - gamma_opt_scenarios(1);
    fprintf('  • Current scenario: %.2f° %s than perfect terrain-following\n', ...
            abs(rad2deg(delta_from_perfect)), ...
            iif(delta_from_perfect < 0, 'smaller', 'larger'));
end

% Warning for negative gamma_max
invalid_scenarios = find(isnan(gamma_opt_scenarios));
if ~isempty(invalid_scenarios)
    fprintf('\n  ⚠ WARNING: ');
    for i = 1:length(invalid_scenarios)
        fprintf('Scenario %d ', invalid_scenarios(i));
    end
    fprintf('have negative γ_max → infeasible configuration!\n');
end

fprintf('\n');

%% ============================================================================
% 14. DISPLAY VERSORS FOR MATLAB CODE GENERATION
%% ============================================================================

fprintf('\n');
fprintf('VERSORS FOR SENSOR CONFIGURATION (Robot Frame):\n');
fprintf('Frame: +X=forward, +Y=right, +Z=down (NED convention)\n\n');

g = -gamma_opt;
l = gamma_opt;
sin_g = sin(g);
cos_g = cos(g);
sin_l = sin(l);
cos_l = cos(l);

fprintf('Final consideration:\n\n');
fprintf('Optimal found gamma = %.2f\n\n', rad2deg(gamma_opt));

fprintf('%% Versors for 4 SBES sensors\n');
fprintf('r_s = zeros(3, 4);\n');
fprintf('r_s(:, 1) = [sin(gamma), 0, cos(gamma)]'';  %% Rear (South)\n');
fprintf('r_s(:, 2) = [sin(lambda), 0, cos(lambda)]'';  %% Front (North)\n');
fprintf('r_s(:, 3) = [0, -sin(eta), cos(eta)]'';   %% Right (East)\n');
fprintf('r_s(:, 4) = [0, -sin(zeta), cos(zeta)]'';  %% Left (West)\n\n');

fprintf('Numeric values:\n');
fprintf('r_s(:, 2) = [%+.6f, %+.6f, %+.6f]''  %% Rear\n', sin_g, 0, cos_g);
fprintf('r_s(:, 1) = [%+.6f, %+.6f, %+.6f]''  %% Front\n', sin_l, 0, cos_l);
fprintf('r_s(:, 3) = [%+.6f, %+.6f, %+.6f]''  %% Right\n', 0, -sin_l, cos_l);
fprintf('r_s(:, 4) = [%+.6f, %+.6f, %+.6f]''  %% Left\n\n', 0, -sin_g, cos_g);

%% ============================================================================
% 15. DEFINE OBJECTIVE FUNCTION
%% ============================================================================

function J = objective_function(gamma, h, v, t_react, ...
                                alpha_terrain_max, beta_terrain_max, robot_roll, robot_pitch, tracking_error, ...
                                s_ref, k_c, gamma_opt_ref, sigma_gamma, ...
                                w1, w2, w3, w4, denom_min_safe, scale_uncertainty)
    % Multi-objective function to MAXIMIZE (3D version)
    % J(γ) = w1·f_res(γ) + w2·f_cov(γ) + w3·f_rob(γ) + w4·f_unc(γ)
    %
    % INPUTS (all angles in radians):
    %   gamma              - SBES sensor angle from vertical
    %   h                  - Altitude above terrain [m]
    %   v                  - Robot velocity [m/s]
    %   t_react            - Reaction time [s]
    %   alpha_terrain_max  - Maximum terrain slope (roll direction) [rad]
    %   beta_terrain_max   - Maximum terrain slope (pitch direction) [rad]
    %   robot_roll         - Robot roll angle φ [rad]
    %   robot_pitch        - Robot pitch angle θ [rad]
    %   tracking_error     - Tracking error δ [rad]
    %   s_ref              - Reference baseline [m]
    %   k_c                - Coverage scaling factor
    %   gamma_opt_ref      - Optimal acoustic angle [rad]
    %   sigma_gamma        - Acoustic bandwidth [rad]
    %   w1, w2, w3, w4     - Component weights
    %   denom_min_safe     - Stability threshold
    %   scale_uncertainty  - Uncertainty scaling
    %
    % OUTPUT:
    %   J - Combined objective value (to be maximized)
    
    % Component 1: Resolution (maximize baseline size) - 3D
    % Baseline projected on terrain plane for both axes
    baseline_robot = 2 * h * tan(gamma);  % Baseline in robot frame
    baseline_eff_pitch = baseline_robot * cos(robot_pitch);  % N/S sensors
    baseline_eff_roll = baseline_robot * cos(robot_roll);    % E/W sensors
    baseline_effective = (baseline_eff_pitch + baseline_eff_roll) / 2;  % Average
    f_res = tanh(baseline_effective / s_ref);
    
    % Component 2: Coverage (look-ahead distance)
    d_ahead = h * tan(gamma);
    f_cov = 1 - exp(-k_c * (d_ahead - v*t_react));
    
    % Component 3: Robustness (acoustic return quality)
    f_rob = exp(-(gamma - gamma_opt_ref)^2 / (2*sigma_gamma^2));
    
    % Component 4: Uncertainty (geometric stability on inclined terrain) - 3D
    % Consider both roll and pitch axes independently
    
    % ROLL axis (sensors E/W): influenced by φ and α
    if sign(robot_roll) == sign(alpha_terrain_max)
        effective_slope_roll = abs(abs(robot_roll) - abs(alpha_terrain_max)) + tracking_error;
    else
        effective_slope_roll = abs(robot_roll) + abs(alpha_terrain_max) + tracking_error;
    end
    
    % PITCH axis (sensors N/S): influenced by θ and β
    if sign(robot_pitch) == sign(beta_terrain_max)
        effective_slope_pitch = abs(abs(robot_pitch) - abs(beta_terrain_max)) + tracking_error;
    else
        effective_slope_pitch = abs(robot_pitch) + abs(beta_terrain_max) + tracking_error;
    end
    
    % Take worst case (most restrictive)
    effective_slope = max(effective_slope_roll, effective_slope_pitch);
    theta_ray_terrain = gamma + effective_slope;
    
    % Denominator represents margin before ray becomes parallel to terrain
    % denom = cos(effective angle) → approaches 0 when angle → 90°
    denom = cos(theta_ray_terrain) / cos(effective_slope);
    
    % Normalize and apply safety threshold
    f_unc = tanh(max(denom, denom_min_safe) / scale_uncertainty);
    
    % Combined objective
    J = w1*f_res + w2*f_cov + w3*f_rob + w4*f_unc;
end

function result = iif(condition, true_val, false_val)
    % Inline if function
    if condition
        result = true_val;
    else
        result = false_val;
    end
end