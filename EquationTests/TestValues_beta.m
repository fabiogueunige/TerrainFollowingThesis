clear; close all; clc;

% Flag per usare angoli specifici o generati randomicamente
use_specific_angles = true; % Imposta a 'false' per angoli randomici
tolerance = 1e-4;
areDifferent = @(a, b, tol) abs(a - b) > tol * max(abs(a), abs(b));

%% Measuraments Variables
pr = [0,0]'; % [x,z]
qt = - 10;
Gamma = -pi/8;
Lambda = pi/8;
u = 10000;
w = 0;
Ts = 0.001;

if use_specific_angles
    beta = pi/7;
    theta = -pi/10;
else
    % random angles generator
    lower_bound = -pi/2;
    upper_bound = pi/2;
    % Genera un singolo angolo randomico
    beta = lower_bound + (upper_bound - lower_bound) * rand();
    lower_bound = -pi/4;
    upper_bound = pi/4;
    theta = lower_bound + (upper_bound - lower_bound) * rand();
end

fprintf('Angoli:\n');
fprintf('beta: %.2f\n', rad2deg(beta));
fprintf('theta: %.2f\n', rad2deg(theta));

fprintf('\nVelocità:\n');
fprintf('u: %.2f\n', u);
fprintf('w: %.2f\n', w);

m1_g = tan((3*pi/2 + Gamma + theta)); 
m2_l = tan((3*pi/2 + Lambda + theta));
    
mt = tan(beta);
mp = tan(theta);

% robot position
q1 = pr(2) - m1_g*pr(1);
q2 = pr(2) - m2_l*pr(1);
qp = pr(2) - mp*pr(1);

z_r = [0, -1]';
z_r = rotY2D(theta)*z_r;

% computation for y1 
xc1 = (q1 - qt) / (mt - m1_g);
zc1 = mt * xc1 + qt;
y1m = norm([pr(1) - xc1; pr(2) - zc1]);
v_p1 = [xc1 - pr(1); zc1 - pr(2)];
visibile_y1 = dot(z_r, v_p1) > 0;
if ~visibile_y1
    error('Errore in visibilità y1 \n');
end

% Computation for y2
xc2 = (q2 - qt) / (mt - m2_l);
zc2 = mt * xc2 + qt;
y2m = norm([pr(1) - xc2; pr(2) - zc2]);
v_p2 = [xc2 - pr(1); zc2 - pr(2)];
visibile_y2 = dot(z_r, v_p2) > 0;

if ~visibile_y2
    error('Errore in visibilità y2 \n');
end

%% valore da ottenere di h
hm = (abs(mt*pr(1) - pr(2) + qt))/ (sqrt(mt^2 + 1));

%% Y computation
y1_abs = hm/abs(cos(Gamma - (beta - theta)));
y1 = hm/(cos(Gamma - (beta - theta)));
y2_abs = hm/abs(cos(Lambda - (beta - theta)));
y2 = hm/(cos(Lambda - (beta - theta)));

%% h check
h_y1 = y1m*(cos(Gamma - (beta - theta)));
h_y2_abs = y2m * abs(cos(Lambda - (beta - theta)));

% for plot
x_dir = rotY2D(theta) * [1; 0]; 
z_dir = rotY2D(theta) * [0; -1];

%% h variation & Motion
dx = u*cos(theta) + w*sin(theta);
dz = -(-u*sin(theta) + w*cos(theta));
pr_new = pr + [dx*Ts, dz*Ts]'; 
h_new = hm - u*sin((beta - theta))*Ts - w*cos((beta - theta))*Ts;
hm_new = (abs(mt*pr_new(1) - pr_new(2) + qt))/ (sqrt(mt^2 + 1));

%% New sensor values
q1_new = pr_new(2) - m1_g*pr_new(1);
q2_new = pr_new(2) - m2_l*pr_new(1);
qp_new = pr_new(2) - mp*pr_new(1);

% computation for y1_new
xc1_new = (q1_new - qt) / (mt - m1_g);
zc1_new = mt * xc1_new + qt;
y1m_new = norm([pr_new(1) - xc1_new; pr_new(2) - zc1_new]);
v_p1 = [xc1_new - pr_new(1); zc1_new - pr_new(2)];
visibile_y1 = dot(z_r, v_p1) > 0;
if ~visibile_y1
    error('Errore in visibilità y1 new \n');
end

% Computation for y2_new
xc2_new = (q2_new - qt) / (mt - m2_l);
zc2_new = mt * xc2_new + qt;
y2m_new = norm([pr_new(1) - xc2_new; pr_new(2) - zc2_new]);
v_p2 = [xc2_new - pr_new(1); zc2_new - pr_new(2)];
visibile_y2 = dot(z_r, v_p2) > 0;

if ~visibile_y2
    error('Errore in visibilità y2 new \n');
end

fprintf('\nValori Calcolati:\n');
fprintf('hm: %.4f\n', hm);
fprintf('y1m: %.4f\n', y1m);
fprintf('y2m: %.4f\n', y2m);

fprintf('\nValori Calcolati nel nuovo punto:\n');
fprintf('hm_new: %.4f\n', hm_new);
fprintf('y1m_new: %.4f\n', y1m_new);
fprintf('y2m_new: %.4f\n', y2m_new);

%% Check
if areDifferent(hm, h_y1, tolerance) || areDifferent(h_y1, h_y2_abs, tolerance)
    fprintf('Errore in h measurament\n');
    fprintf('hm: %.4f\n', hm);
    fprintf('h_y1: %.4f\n', h_y1);
    fprintf('h_y2_abs: %.4f\n', h_y2_abs);
end

if areDifferent(y1m, y1_abs, tolerance) || areDifferent(y1_abs, y1, tolerance)
    fprintf('Errore in y1\n');
    fprintf('y1m: %.4f\n', y1m);
    fprintf('y1_abs: %.4f\n', y1_abs);
    fprintf('y1: %.4f\n', y1);
end

if areDifferent(y2m, y2_abs, tolerance) || areDifferent(y2_abs, y2, tolerance)
    fprintf('Errore in y2 \n');
    fprintf('y2m: %.4f\n', y2m);
    fprintf('y2_abs: %.4f\n', y2_abs);
    fprintf('y2: %.4f\n', y2);
end

if areDifferent(hm_new, h_new, tolerance)
    fprintf('Controllo velocità e valori h\n');
    fprintf('Errore nella variazione di h \n');
    fprintf('h dalle velocità: %.4f\n', h_new);
    val_u = -u*sin((beta - theta))*Ts;
    val_w = -w*cos((beta - theta))*Ts;
    fprintf('velocità u: %.4f, velocità w: %.4f\n', val_u, val_w);
end

%% Plot
% Intervallo di x per il plot
x_min = min([pr(1), xc1_new, xc2_new, xc1, xc2]) - 5;
x_max = max([pr(1), xc1_new, xc2_new, xc1, xc2]) + 5;
x_values = linspace(x_min, x_max, 100);

% Equazioni delle rette
y_mt = mt * x_values + qt;
y_mp = mp * x_values + qp;
y_mp_new = mp * x_values + qp_new;

% plot per h
m_perp = -1/mt; 
b_perp = pr(2) - m_perp * pr(1); 
x_hm = (b_perp - qt) / (mt - m_perp);
y_hm = mt * x_hm + qt;

% plot per h_new
m_perp = -1/mt; 
b_perp_new = pr_new(2) - m_perp * pr_new(1); 
x_hm_new = (b_perp_new - qt) / (mt - m_perp);
y_hm_new = mt * x_hm_new + qt;

figure;
hold on;
grid on;
box on;

plot(x_values, y_mt, 'y-', 'LineWidth', 3, 'DisplayName', 'Retta terrain (mt)');
plot([pr(1), xc1], [pr(2), zc1], 'r-', 'LineWidth', 1.5, 'DisplayName', 'Retta y1 (m1_g)');
plot([pr(1), xc2], [pr(2), zc2], 'b-', 'LineWidth', 1.5, 'DisplayName', 'Retta y2 (m2_l)');
plot([pr_new(1), xc1_new], [pr_new(2), zc1_new], 'r-', 'LineWidth', 1.5, 'DisplayName', 'Retta y1 new');
plot([pr_new(1), xc2_new], [pr_new(2), zc2_new], 'b-', 'LineWidth', 1.5, 'DisplayName', 'Retta y2 new)');
% old point
t_tmp = 3;
p_x_dir = pr + t_tmp * x_dir;
p_z_dir = pr + t_tmp * z_dir;
plot([pr(1), p_x_dir(1)], [pr(2), p_x_dir(2)], 'r--', 'LineWidth', 1, 'DisplayName', 'Asse X rob');
plot([pr(1), p_z_dir(1)], [pr(2), p_z_dir(2)], 'b--', 'LineWidth', 1, 'DisplayName', 'Asse Z rob');
% new point
x_dir_new = rotY2D(theta) * [1; 0]; 
z_dir_new = rotY2D(theta) * [0; -1];
p_x_dir = pr_new + t_tmp * x_dir_new;
p_z_dir = pr_new + t_tmp * z_dir_new;
plot([pr_new(1), p_x_dir(1)], [pr_new(2), p_x_dir(2)], 'r--', 'LineWidth', 1, 'DisplayName', 'Asse X rob');
plot([pr_new(1), p_z_dir(1)], [pr_new(2), p_z_dir(2)], 'b--', 'LineWidth', 1, 'DisplayName', 'Asse Z rob');


% Plot dei punti noti
plot(pr(1), pr(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Punto Pr (0,0)');
plot(xc1, zc1, 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Int xc1,zc1');
plot(xc2, zc2, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Int xc2,zc2');
plot(xc1_new, zc1_new, 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'New Int xc1,zc1');
plot(xc2_new, zc2_new, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'New Int xc2,zc2');
plot(pr_new(1), pr_new(2), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Punto Pr nuovo');

plot(x_hm, y_hm, 'kd', 'MarkerSize', 10, 'DisplayName', 'Punto hm new sul Terreno'); 
plot([pr(1), x_hm], [pr(2), y_hm], 'k:', 'LineWidth', 1.5, 'DisplayName', 'Segmento hm'); 

plot(x_hm_new, y_hm_new, 'kd', 'MarkerSize', 10, 'DisplayName', 'Punto hm sul Terreno'); 
plot([pr_new(1), x_hm_new], [pr_new(2), y_hm_new], 'k:', 'LineWidth', 1.5, 'DisplayName', 'Segmento hm');

% Aggiungi etichette e legenda
xlabel('Asse X');
ylabel('Asse Z (val neg)');
title('Plot delle Rette nel Piano');
legend('Location', 'best');
axis equal; % Assicura che le unità sugli assi siano uguali per evitare distorsioni angolari
ylim_auto = [min([min(y_mt), pr(1), pr_new(1), qt, zc1_new, zc2_new])-5, max([max(y_mt), pr(1), pr_new(1), qt, zc1_new, zc2_new])+5];
if isfinite(ylim_auto(1)) && isfinite(ylim_auto(2))
    ylim(ylim_auto);
end
hold off;



function R = rotY2D(a)
   R = [cos(a), -sin(a);
      sin(a),  cos(a)];
end