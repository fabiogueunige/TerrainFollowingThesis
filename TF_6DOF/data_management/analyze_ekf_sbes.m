% Quick analysis of EKF SBES behavior
% Runs simulation and analyzes why est vs robot parallel is low

close all; clear; clc;

% Run simulation
main_6DOF_3D;

%% Analysis
fprintf('\n=== EKF SBES Altitude Analysis ===\n');
alt = x_ekf_sbes(1,:);
fprintf('Altitude: min=%.3f, max=%.3f, mean=%.3f, std=%.3f\n', ...
    min(alt), max(alt), mean(alt), std(alt));
fprintf('Samples with alt < 0.7 (emergency threshold): %d / %d\n', sum(alt < 0.7), length(alt));
fprintf('Samples with alt < 1.0: %d / %d\n', sum(alt < 1.0), length(alt));

fprintf('\n=== EKF SBES Angles Analysis ===\n');
alpha = rad2deg(x_ekf_sbes(2,:));
beta = rad2deg(x_ekf_sbes(3,:));
fprintf('Alpha (pitch-related): min=%.1f, max=%.1f, mean=%.1f, std=%.1f deg\n', ...
    min(alpha), max(alpha), mean(alpha), std(alpha));
fprintf('Beta (roll-related): min=%.1f, max=%.1f, mean=%.1f, std=%.1f deg\n', ...
    min(beta), max(beta), mean(beta), std(beta));

fprintf('\n=== Robot Angles ===\n');
roll_deg = rad2deg(eta_gt(4,:));
pitch_deg = rad2deg(eta_gt(5,:));
yaw_deg = rad2deg(eta_gt(6,:));
fprintf('Roll: min=%.1f, max=%.1f, mean=%.1f deg\n', min(roll_deg), max(roll_deg), mean(roll_deg));
fprintf('Pitch: min=%.1f, max=%.1f, mean=%.1f deg\n', min(pitch_deg), max(pitch_deg), mean(pitch_deg));
fprintf('Yaw: min=%.1f, max=%.1f, mean=%.1f deg\n', min(yaw_deg), max(yaw_deg), mean(yaw_deg));

fprintf('\n=== Goal Angles ===\n');
goal_roll = rad2deg([goal.roll]);
goal_pitch = rad2deg([goal.pitch]);
fprintf('Goal Roll: min=%.1f, max=%.1f, mean=%.1f deg\n', min(goal_roll), max(goal_roll), mean(goal_roll));
fprintf('Goal Pitch: min=%.1f, max=%.1f, mean=%.1f deg\n', min(goal_pitch), max(goal_pitch), mean(goal_pitch));

fprintf('\n=== State Machine Analysis ===\n');
% Count unique states
unique_states = unique({state_m{:}});
fprintf('States encountered: ');
for i = 1:length(unique_states)
    state_count = sum(strcmp(state_m, unique_states{i}));
    fprintf('%s=%d ', unique_states{i}, state_count);
end
fprintf('\n');

%% Plot comparison
figure('Name', 'EKF SBES vs Robot Analysis');

subplot(3,1,1);
plot(t, alt, 'b-');
hold on;
yline(0.7, 'r--', 'Emergency threshold');
yline(3, 'g--', 'Target altitude');
title('EKF SBES Estimated Altitude');
ylabel('Altitude [m]');
xlabel('Time [s]');
legend('EKF altitude', 'Emergency', 'Target');
grid on;

subplot(3,1,2);
plot(t, alpha, 'b-', t, pitch_deg, 'r--');
title('Alpha vs Robot Pitch');
ylabel('Angle [deg]');
xlabel('Time [s]');
legend('EKF alpha', 'Robot pitch');
grid on;

subplot(3,1,3);
plot(t, beta, 'b-', t, roll_deg, 'r--');
title('Beta vs Robot Roll');
ylabel('Angle [deg]');
xlabel('Time [s]');
legend('EKF beta', 'Robot roll');
grid on;

%% Analysis of normal vectors alignment
fprintf('\n=== Normal Vector Computation Check ===\n');
% At sample 25000 (mid-simulation)
k = 25000;
fprintf('At t=%.1f s:\n', t(k));
fprintf('  EKF alpha=%.2f deg, beta=%.2f deg\n', rad2deg(x_ekf_sbes(2,k)), rad2deg(x_ekf_sbes(3,k)));
fprintf('  Robot roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg\n', ...
    rad2deg(eta_gt(4,k)), rad2deg(eta_gt(5,k)), rad2deg(eta_gt(6,k)));
fprintf('  Goal roll=%.2f deg, pitch=%.2f deg\n', rad2deg(goal(k).roll), rad2deg(goal(k).pitch));

% Compute normal from EKF
alpha_k = x_ekf_sbes(2,k);
beta_k = x_ekf_sbes(3,k);
n_ekf = [-sin(alpha_k); sin(beta_k)*cos(alpha_k); cos(alpha_k)*cos(beta_k)];
n_ekf = n_ekf / norm(n_ekf);

% Robot z-axis in world frame (from rotation matrix)
phi = eta_gt(4,k); theta = eta_gt(5,k); psi = eta_gt(6,k);
Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
R = Rz * Ry * Rx;
n_robot = R(:,3);  % z-axis of robot in world frame

fprintf('  EKF normal (world): [%.3f, %.3f, %.3f]\n', n_ekf(1), n_ekf(2), n_ekf(3));
fprintf('  Robot z-axis (world): [%.3f, %.3f, %.3f]\n', n_robot(1), n_robot(2), n_robot(3));
fprintf('  Angle between: %.2f deg\n', rad2deg(acos(abs(dot(n_ekf, n_robot)))));
