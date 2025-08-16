clear; close all; clc;

% Flag per usare angoli specifici o generati randomicamente
use_specific_angles = true; % Imposta a 'false' per angoli randomici
use_specific_speed = true; % Imposta a 'false' per velocità randomiche
tolerance = 1e-4;
areDifferent = @(a, b, tol) abs(a - b) > tol * max(abs(a), abs(b));

%% Simulation parameters
Ts = 0.001;
psi = 0;

%% Angle defintions
if use_specific_angles
    % terrain
    beta = pi/4;
    alpha = pi/4;
    % robot
    theta = 0;
    phi = pi/9;
else
    % random angles generator
    lower_bound = -pi/3;
    upper_bound = pi/3;
    % Genera un singolo angolo randomico
    beta = lower_bound + (upper_bound - lower_bound) * rand();
    alpha = lower_bound + (upper_bound - lower_bound) * rand();
    lower_bound = -pi/5;
    upper_bound = pi/5;
    theta = lower_bound + (upper_bound - lower_bound) * rand();
    phi = lower_bound + (upper_bound - lower_bound) * rand();
end
fprintf('Valori per angoli\n');
fprintf('beta: %.2f\n', rad2deg(beta));
fprintf('alpha: %.2f\n', rad2deg(alpha));
fprintf('theta: %.2f\n', rad2deg(theta));
fprintf('phi: %.2f\n', rad2deg(phi)); 

%% speed definitions
if use_specific_speed
    surge = 1000;
    sway = 1000;
    heave = 0;
    p = 0;
    q = 0; 
else
    % random angles generator
    lower_bound = -2000;
    upper_bound = 2000;
    % Genera un singolo angolo randomico
    surge = lower_bound + (upper_bound - lower_bound) * rand();
    sway = lower_bound + (upper_bound - lower_bound) * rand();
    heave = lower_bound + (upper_bound - lower_bound) * rand();
    lower_bound = -(pi/8)/Ts;
    upper_bound = (pi/8)/Ts;
    p = lower_bound + (upper_bound - lower_bound) * rand();
    q = lower_bound + (upper_bound - lower_bound) * rand();
end
fprintf('\nValori per velocità\n');
fprintf('u: %.2f\n', surge);
fprintf('v: %.2f\n', sway);
fprintf('w: %.2f\n', heave);
fprintf('p: %.2f\n', rad2deg(p)); 
fprintf('q: %.2f\n', rad2deg(q)); 

%% Terrain Parameters
pplane = [0, 0, 15]';
n0 = [0, 0, 1]'; % in terrain frame!!
% Transformation (given by the sensors)
wRt = rotz(0)*roty(beta)*rotx(alpha)*rotx(pi);

%% Robot Parameters
pr = [0, 0, 0]';
num_s = 4;
Gamma = -pi/8; % y1 angle (rear) sud
Lambda = pi/8; % y2 angle (front) nord
Eta = pi/8; % y3 angle (left) ovest
Zeta = -pi/8; % y4 angle (right) est

r_speed = [surge, sway, heave, p, q, 0]'; 
% Trasformation with robot frame
wRr = rotz(psi)*roty(theta)*rotx(phi);

% for plot robot frame to world frame
x_dir = wRr * [1; 0; 0];
y_dir = wRr * [0; 1; 0];
z_dir = wRr * [0; 0; 1];

r_s = zeros(3,num_s);
s = r_s;
r_s(:, 1) = [sin(Gamma), 0, cos(Gamma)]';
r_s(:, 2) = [sin(Lambda), 0, cos(Lambda)]';
r_s(:, 3) = [0, -sin(Eta), cos(Eta)]';
r_s(:, 4) = [0, -sin(Zeta), cos(Zeta)]';
% check of consistency for the sensors
% Trasformation in world frame
for k = 1:num_s
    s(:,k) = wRr*r_s(:,k);
    if (norm(s(:,k)) ~= 1)
        fprintf('ALERT: norm sensor %.0f is not 1\n', k);
        s(:,k) = vector_normalization(s(:,k));
    end
end

%% Geometry Compuataion terrain
%  normal to the plane
n = wRt*n0; % in world frame
fprintf('\nVettore superficie\n');
fprintf('n: [%.4f; %.4f; %.4f]\n', n(1), n(2), n(3));
if (norm(n) ~= 1)
    fprintf('ALERT: norm n is not 1\n');
    n = vector_normalization(n);
end

%% Sensor Values
t_star = zeros(1, num_s);
p_int = zeros(3, num_s);
y = zeros(1, num_s);
for k = 1:num_s
    if dot(s(:,k),n) == 0
        error('The line s and the plane n are parallel');
    end
    t_star(:, k) = -(dot((pr - pplane),n))/(dot(s(:,k),n));
    if t_star(:,k) < 0
        error('Not visible sensor %.0f\n', k);
    end
    p_int(:, k) = pr + t_star(:, k)*s(:, k);
    y(k) = norm(t_star(:, k));
end

%% Altitude Computation
% real altitude in 
h_real = abs((n'*(pr - pplane))/(norm(n)));
fprintf('\nValore per altezza nel primo punto\n');
fprintf('h: %.4f\n', h_real);

%% Check of the results first point
% Problema trovato -> h dalla y viene negativo !! Denominatore è negativo !!
fprintf('\nValori dei sensori per primo punto\n');
y_mes = zeros(1, num_s);
for k = 1:num_s
    fprintf('y%.0f = %.4f\n', k, y(k));
    y_mes(k) = - h_real / (n' * s(:,k));
    if areDifferent(y_mes(k), y(:, k), tolerance)
        fprintf('y%.0f mesured: %.4f\n', k, y_mes(k));
        fprintf('ERRORE: y per il sensore %0.f\n', k);
    end
end

%% Muovo il robot
w_speed = wRr * r_speed(1:3);
pr_new = pr + w_speed*Ts;
% Angles update
phi_old = phi;
theta_old = theta;
phi = phi + r_speed(4)*Ts;
theta = theta + r_speed(5)*Ts; % !!! SEGNOOO !!!!
% Transformation update
wRr = rotz(psi)*roty(theta)*rotx(phi);

%% New Sensor Values
t_star_new = zeros(1, num_s);
p_int_new = zeros(3, num_s);
y_new = zeros(1, num_s);
% Trasformation in world frame
for k = 1:num_s
    s(:,k) = wRr*r_s(:,k);
end
for k = 1:num_s
    if dot(s(:,k),n) == 0
        error('The line s and the plane n are parallel in the new point');
    end
    t_star_new(:, k) = -(dot((pr_new - pplane),n))/(dot(s(:,k),n));
    if t_star_new(:,k) < 0
        error('New t_star value is negative, sensor %.0f', k);
    end
    p_int_new(:, k) = pr_new + t_star_new(:, k)*s(:, k);
    y_new(k) = norm(t_star_new(:, k));
end

%% New Altitude Computation
% real altitude
% ---- Il meno perchè devo considerarlo nell'altro lato ----%
h_real_new = (n'*(pr_new - pplane))/(norm(n));
fprintf('\nValore per altezza nel nuovo punto\n');
fprintf('New h: %.4f\n', h_real_new);

%% Check of the results for consistency
% y Values new
fprintf('\nValori dei sensori per nuovo punto\n');
y_mes_new = zeros(1, num_s);
for k = 1:num_s
    fprintf('New y%.0f = %.4f\n', k, y_new(k));
    y_mes_new(k) = -h_real_new / (n' * s(:,k));
    if areDifferent(y_mes_new(k), y_new(k), tolerance)
        fprintf('New y%0.f mesured was: %.4f\n',k, y_mes_new(k));
        error('Errore in y per il sensore %0.f\n', k);
    end
end

%% Check of the results the states
% controllo senza terreno
s_speed = (wRt)' * w_speed;
h_rot = h_real + s_speed(3) * Ts; 
if areDifferent(h_real_new, h_rot, tolerance)
    fprintf('h from rotation matrices: %.4f\n', h_rot);
    error('Errore nel calcolo della differenza di altezza');
end
if s_speed(3) < 0
    fprintf('h diminuisce di %.4f\n', -s_speed(3)*Ts);
else
    fprintf('h aumenta di %.4f\n', s_speed(3)*Ts);
end
if areDifferent(s_speed(3)*Ts, (h_real_new - h_real), tolerance)
    fprintf('ERROR: Non stesso valore di cambiamento\n');
    fprintf('Cambiamento di h reale %.4f\n', (h_real_new - h_real))
    fprintf('h calcolato corrisponde a %.4f\n', (s_speed(3)*Ts))
end

%% Plot
figure;
hold on;
grid on;
box on;
% for plot
p_proj = pr - dot(n, pr - pplane) * n; % h projection
colors = lines(num_s);

% Plane plot
[xp, yp] = meshgrid(-17:0.5:17, -17:0.5:17);
if abs(n(3)) > 1e-6
    fprintf('\n!!!Attivazione if disegno del piano!!!\n');
    zp = (-n(1)*(xp-pplane(1)) - n(2)*(yp-pplane(2)))/n(3) + pplane(3);
    surf(xp, yp, zp, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'DisplayName', 'Piano');
end

% Robot frame
t_tmp = 3;
p_x_dir = pr + t_tmp * x_dir;
p_y_dir = pr + t_tmp * y_dir;
p_z_dir = pr + t_tmp * z_dir;
% Robot frame plot
plot3([pr(1), p_x_dir(1)], [pr(2), p_x_dir(2)], [pr(3), p_x_dir(3)], 'r--', 'LineWidth', 2, 'DisplayName', 'Asse X rob');
plot3([pr(1), p_y_dir(1)], [pr(2), p_y_dir(2)], [pr(3), p_y_dir(3)], 'g--', 'LineWidth', 2, 'DisplayName', 'Asse Y rob');
plot3([pr(1), p_z_dir(1)], [pr(2), p_z_dir(2)], [pr(3), p_z_dir(3)], 'b--', 'LineWidth', 2, 'DisplayName', 'Asse Z rob');

% New point robot frame
x_dir_new = rotz(psi) * roty(theta) * rotx(phi) * [1; 0; 0];
y_dir_new = rotz(psi) * roty(theta) * rotx(phi) * [0; 1; 0];
z_dir_new = rotz(psi) * roty(theta) * rotx(phi) * [0; 0; 1];
t_tmp = 3;
p_x_dir = pr_new + t_tmp * x_dir_new;
p_y_dir = pr_new + t_tmp * y_dir_new;
p_z_dir = pr_new + t_tmp * z_dir_new;
plot3([pr_new(1), p_x_dir(1)], [pr_new(2), p_x_dir(2)], [pr_new(3), p_x_dir(3)], 'r--', 'LineWidth', 2, 'DisplayName', 'Asse X new rob');
plot3([pr_new(1), p_y_dir(1)], [pr_new(2), p_y_dir(2)], [pr_new(3), p_y_dir(3)], 'g--', 'LineWidth', 2, 'DisplayName', 'Asse Y new rob');
plot3([pr_new(1), p_z_dir(1)], [pr_new(2), p_z_dir(2)], [pr_new(3), p_z_dir(3)], 'b--', 'LineWidth', 2, 'DisplayName', 'Asse Z new rob');

% Sensor plot
for k = 1:num_s
    plot3([pr(1), p_int(1, k)], [pr(2), p_int(2, k)], [pr(3), p_int(3, k)], ...
          'LineWidth', 2, 'Color', colors(k, :), ...
          'DisplayName', sprintf('y%d', k));
    plot3([pr_new(1), p_int_new(1, k)], [pr_new(2), p_int_new(2, k)], [pr_new(3), p_int_new(3, k)], ...
          'LineWidth', 2, 'Color', colors(k, :), ...
          'DisplayName', sprintf('y %d', k));
    plot3(p_int(1, k), p_int(2, k), p_int(3, k), 'x', 'MarkerFaceColor', colors(k, :), ...
            'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', sprintf('Intersezione y%d', k));
    plot3(p_int_new(1, k), p_int_new(2, k), p_int_new(3, k), 'x', 'MarkerFaceColor', colors(k, :), ...
            'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('Intersezione y%d', k));
end

% Punto iniziale
plot3(pr(1), pr(2), pr(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Punto iniziale');
plot3([pr(1), p_proj(1)], [pr(2), p_proj(2)], [pr(3), p_proj(3)], ...
      'k--', 'LineWidth', 2, 'DisplayName', 'Altitude h)');
plot3(p_proj(1), p_proj(2), p_proj(3), 'kd', 'MarkerSize', 6, 'LineWidth', 2, 'DisplayName', 'Point of proj h new');

% Nuovo punto robot
plot3(pr_new(1), pr_new(2), pr_new(3), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Punto Pr nuovo');

% New projection point
p_proj_new = pr_new - dot(n, pr_new - pplane) * n; % h projection
if areDifferent(norm(p_proj_new - pr_new), abs(h_real_new), tolerance)
    fprintf('P projection x: %.0f, y: %.0f, z: %.0f\n', p_proj_new(1), p_proj_new(2), p_proj_new(3));
    fprintf('Valore h = %.4f projected\n', norm(p_proj_new - pr_new));
    error('Error nel calcolo della h nuova\n');
end

plot3([pr_new(1), p_proj_new(1)], [pr_new(2), p_proj_new(2)], [pr_new(3), p_proj_new(3)], ...
      'k--', 'LineWidth', 2, 'DisplayName', 'Altitude h)');
plot3(p_proj_new(1), p_proj_new(2), p_proj_new(3), 'kd', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Point of proj h new');

% Normal to the plane
quiver3(p_proj_new(1), p_proj_new(2), p_proj_new(3), n(1), n(2), n(3), 'r', 'LineWidth', 3, 'MaxHeadSize', 5);
quiver3(p_proj(1), p_proj(2), p_proj(3), n(1), n(2), n(3), 'r', 'LineWidth', 3, 'MaxHeadSize', 5);

xlabel('Asse X');
ylabel('Asse Y');
zlabel('Asse Z');

set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');

title('AUV Situation with Sensors');
% legend('Location', 'best');
axis equal; 
hold off;

if areDifferent(s_speed(3)*Ts, (h_real_new - h_real), tolerance)
    error('Cambiamento nelle altezze');
end

function Rx = rotx(a)
    Rx = [1, 0, 0;
          0, cos(a), -sin(a);
          0, sin(a), cos(a)];
end
function Ry = roty(a)
    Ry = [cos(a), 0, sin(a);
           0, 1, 0;
          -sin(a), 0, cos(a)];
end
function Rz = rotz(a)
    Rz = [cos(a), -sin(a), 0;
          sin(a), cos(a), 0;
            0, 0, 1];
end

function vect = vector_normalization(t)
    vect = t / norm(t);
end
