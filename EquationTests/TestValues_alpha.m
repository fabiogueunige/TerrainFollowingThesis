clear; close all; clc;

% Flag per usare angoli specifici o generati randomicamente
use_specific_angles = true; % Imposta a 'false' per angoli randomici
tolerance = 1e-4;
areDifferent = @(a, b, tol) abs(a - b) > tol * max(abs(a), abs(b));

%% Measuraments Variables
pr = [0,0]'; % [y,z]
qt = - 10;
Eta = -pi/8;
Zeta = pi/8;
v = 10000;
w = 0;
Ts = 0.001;

if use_specific_angles
    alpha = pi/10;
    phi = -pi/10;
else
    % random angles generator
    lower_bound = -pi/2;
    upper_bound = pi/2;
    % Genera un singolo angolo randomico
    alpha = lower_bound + (upper_bound - lower_bound) * rand();
    lower_bound = -pi/4;
    upper_bound = pi/4;
    phi = lower_bound + (upper_bound - lower_bound) * rand();
end

fprintf('Angoli:\n');
fprintf('alpha: %.2f\n', rad2deg(alpha));
fprintf('phi: %.2f\n', rad2deg(phi));

fprintf('\nVelocità:\n');
fprintf('v: %.2f\n', v);
fprintf('w: %.2f\n', w);

m3_e = tan((3*pi/2 + Eta + phi)); 
m4_z = tan((3*pi/2 + Zeta + phi));
    
mt = tan(alpha);
mr = tan(phi);

% robot position
q3 = pr(2) - m3_e*pr(1);
q4 = pr(2) - m4_z*pr(1);
qr = pr(2) - mr*pr(1);

z_r = [0, -1]';
z_r = rotX2D(phi)*z_r;

% computation for y1 
yc3 = (q3 - qt) / (mt - m3_e);
zc3 = mt * yc3 + qt;
y3m = norm([pr(1) - yc3; pr(2) - zc3]);
v_p3 = [yc3 - pr(1); zc3 - pr(2)];
visibile_y3 = dot(z_r, v_p3) > 0;
if ~visibile_y3
    error('Errore in visibilità y3 \n');
end

% Computation for y2
yc4 = (q4 - qt) / (mt - m4_z);
zc4 = mt * yc4 + qt;
y4m = norm([pr(1) - yc4; pr(2) - zc4]);
v_p4 = [yc4 - pr(1); zc4 - pr(2)];
visibile_y4 = dot(z_r, v_p4) > 0;

if ~visibile_y4
    error('Errore in visibilità y4 \n');
end

%% valore da ottenere di h
hm = (abs(mt*pr(1) - pr(2) + qt))/ (sqrt(mt^2 + 1));

%% Y computation
y3_abs = hm/abs(cos(Eta - (alpha - phi)));
y3 = hm/(cos(Eta - (alpha - phi)));
y4_abs = hm/abs(cos(Zeta - (alpha - phi)));
y4 = hm/(cos(Zeta - (alpha - phi)));

%% h check
h_y3 = y3m*(cos(Eta - (alpha - phi)));
h_y4_abs = y4m * abs(cos(Zeta - (alpha - phi)));
% for plot 
y_dir = rotX2D(phi) * [-1; 0]; 
z_dir = rotX2D(phi) * [0; -1];
%% h variation & Motion
dy = -(v*cos(phi) - w*sin(phi));
dz = -(v*sin(phi) + w*cos(phi));
pr_new = pr + [dy*Ts, dz*Ts]'; 
h_new = hm + v*sin((alpha - phi))*Ts - w*cos((alpha - phi))*Ts;
hm_new = (abs(mt*pr_new(1) - pr_new(2) + qt)) / (sqrt(mt^2 + 1));

%% New sensor values
q3_new = pr_new(2) - m3_e*pr_new(1);
q4_new = pr_new(2) - m4_z*pr_new(1);
qr_new = pr_new(2) - mr*pr_new(1);

% computation for y3_new
yc3_new = (q3_new - qt) / (mt - m3_e);
zc3_new = mt * yc3_new + qt;
y3m_new = norm([pr_new(1) - yc3_new; pr_new(2) - zc3_new]);
v_p3 = [yc3_new - pr_new(1); zc3_new - pr_new(2)];
visibile_y3 = dot(z_r, v_p3) > 0;
if ~visibile_y3
    error('Errore in visibilità y3 new\n');
end

% Computation for y4_new
yc4_new = (q4_new - qt) / (mt - m4_z);
zc4_new = mt * yc4_new + qt;
y4m_new = norm([pr_new(1) - yc4_new; pr_new(2) - zc4_new]);
v_p4 = [yc4_new - pr_new(1); zc4_new - pr_new(2)];
visibile_y4 = dot(z_r, v_p4) > 0;

if ~visibile_y4
    error('Errore in visibilità y4 new \n');
end

fprintf('\nValori Calcolati:\n');
fprintf('hm: %.4f\n', hm);
fprintf('y3m: %.4f\n', y3m);
fprintf('y4m: %.4f\n', y4m);

fprintf('\nValori Calcolati nel nuovo punto:\n');
fprintf('hm_new: %.4f\n', hm_new);
fprintf('y3m_new: %.4f\n', y3m_new);
fprintf('y4m_new: %.4f\n', y4m_new);

%% Check
if areDifferent(hm, h_y3, tolerance) || areDifferent(h_y3, h_y4_abs, tolerance)
    fprintf('Errore in h measurament\n');
    fprintf('hm: %.4f\n', hm);
    fprintf('h_y3: %.4f\n', h_y3);
    fprintf('h_y4_abs: %.4f\n', h_y4_abs);
end

if areDifferent(y3m, y3_abs, tolerance) || areDifferent(y3_abs, y3, tolerance)
    fprintf('Errore in y3\n');
    fprintf('y3m: %.4f\n', y3m);
    fprintf('y3_abs: %.4f\n', y3_abs);
    fprintf('y3: %.4f\n', y3);
end

if areDifferent(y4m, y4_abs, tolerance) || areDifferent(y4_abs, y4, tolerance)
    fprintf('Errore in y4 \n');
    fprintf('y4m: %.4f\n', y4m);
    fprintf('y4_abs: %.4f\n', y4_abs);
    fprintf('y4: %.4f\n', y4);
end

if areDifferent(hm_new, h_new, tolerance)
    fprintf('Controllo velocità e valori h\n');
    fprintf('Errore nella variazione di h \n');
    fprintf('h dalle velocità: %.4f\n', h_new);
    val_v = -v*sin((alpha - phi))*Ts;
    val_w = -w*cos((alpha - phi))*Ts;
    fprintf('velocità u: %.4f, velocità w: %.4f\n', val_v, val_w);
end

%% Plot
% Intervallo di x per il plot
y_min = min([pr(1), yc3_new, yc4_new, yc3, yc4]) - 5;
y_max = max([pr(1), yc3_new, yc4_new, yc3, yc4]) + 5;
y_values = linspace(y_min, y_max, 100);

% Equazioni delle rette
y_mt = mt * y_values + qt;
z_mr = mr * y_values + qr;
z_mr_new = mr * y_values + qr_new;

% plot per h
m_perp = -1/mt; 
b_perp_new = pr(2) - m_perp * pr(1); 
y_hm = (b_perp_new - qt) / (mt - m_perp);
z_hm = mt * y_hm + qt;

% plot per h new
m_perp = -1/mt; 
b_perp_new = pr_new(2) - m_perp * pr_new(1); 
y_hm_new = (b_perp_new - qt) / (mt - m_perp);
z_hm_new = mt * y_hm_new + qt;

figure;
hold on;
grid on;
box on;

plot(y_values, y_mt, 'y-', 'LineWidth', 3, 'DisplayName', 'terrain');
plot([pr(1), yc3], [pr(2), zc3], 'c-', 'LineWidth', 1.5, 'DisplayName', 'y3');
plot([pr(1), yc4], [pr(2), zc4], 'm-', 'LineWidth', 1.5, 'DisplayName', 'y4');
plot([pr_new(1), yc3_new], [pr_new(2), zc3_new], 'c-', 'LineWidth', 1.5, 'DisplayName', 'y3 new');
plot([pr_new(1), yc4_new], [pr_new(2), zc4_new], 'm-', 'LineWidth', 1.5, 'DisplayName', 'y4 new');

% old point
t_tmp = 3;
p_y_dir = pr + t_tmp * y_dir;
p_z_dir = pr + t_tmp * z_dir;
plot([pr(1), p_y_dir(1)], [pr(2), p_y_dir(2)], 'g--', 'LineWidth', 1, 'DisplayName', 'Asse Y rob');
plot([pr(1), p_z_dir(1)], [pr(2), p_z_dir(2)], 'b--', 'LineWidth', 1, 'DisplayName', 'Asse Z rob');
% new point
y_dir_new = rotX2D(phi) * [-1; 0]; 
z_dir_new = rotX2D(phi) * [0; -1];
p_y_dir = pr_new + t_tmp * y_dir_new;
p_z_dir = pr_new + t_tmp * z_dir_new;
plot([pr_new(1), p_y_dir(1)], [pr_new(2), p_y_dir(2)], 'g--', 'LineWidth', 1, 'DisplayName', 'Asse Y rob');
plot([pr_new(1), p_z_dir(1)], [pr_new(2), p_z_dir(2)], 'b--', 'LineWidth', 1, 'DisplayName', 'Asse Z rob');

% Plot dei punti noti
plot(pr(1), pr(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Initial Pr (0,0)');
plot(yc3, zc3, 'cx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Int yc3,zc4');
plot(yc4, zc4, 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Int yc4,zc4');
plot(yc3_new, zc3_new, 'cx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'New Int yc3,zc3');
plot(yc4_new, zc4_new, 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'New Int yc4,zc4');
plot(pr_new(1), pr_new(2), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 10, 'DisplayName', 'New Pr');

plot(y_hm, z_hm, 'kd', 'MarkerSize', 10, 'DisplayName', 'Punto hm sul Terreno'); % Punto hm
plot([pr(1), y_hm], [pr(2), z_hm], 'k:', 'LineWidth', 1.5, 'DisplayName', 'Segmento hm'); % Retta congiungente

plot(y_hm_new, z_hm_new, 'kd', 'MarkerSize', 10, 'DisplayName', 'Punto hm_new sul Terreno'); % Punto hm
plot([pr_new(1), y_hm_new], [pr_new(2), z_hm_new], 'k:', 'LineWidth', 1.5, 'DisplayName', 'Segmento hm_new'); % Retta congiungente

% Aggiungi etichette e legenda
xlabel('Asse Y');
ylabel('Asse Z (val neg)');
title('Plot delle Rette nel Piano');
legend('Location', 'best');
axis equal; % Assicura che le unità sugli assi siano uguali per evitare distorsioni angolari
hold off;

function R = rotX2D(a)
   R = [cos(a), -sin(a);
      sin(a),  cos(a)];
end
