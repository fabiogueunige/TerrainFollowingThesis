%% RUN only after Kal_path_mat & Model_equation_mat
% Surge PI
Kd_surge = 0;
Kp_surge = 2*damp*wn*mv(1) - dv(1); % 17.3
Ki_surge = wn^2 * mv(1); % 55.4
ti_surge = Kp_surge / Ki_surge;
Kt_surge = 1 / ti_surge;

% DEPTH (pid)
Kd_dep = (p + 2*damp*wn)*mv(3) + dv(3);
Kp_dep = mv(3)*((wn^2) + 2*damp*p*wn);
Ki_dep = p*(wn^2)*mv(3);
ti_dep = Kp_dep / Ki_dep;
td_dep = Kd_dep / Kp_dep;
Kt_dep = 1 / sqrt(ti_dep*td_dep);

% % theta (PID)
% Kd_theta = (p + 2*damp*wn)*mv(5) - dv(5); 
% Kp_theta = mv(5)*((wn^2) + 2*damp*p*wn); 
% Ki_theta = p*(wn^2)*mv(5);
% ti_theta = Kp_theta / Ki_theta;
% td_theta = Kd_theta / Kp_theta;
% Kt_theta = 1 / sqrt(ti_theta*td_theta);

