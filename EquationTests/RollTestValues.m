clear; close all; clc;

% Flag per usare angoli specifici o generati randomicamente
use_specific_angles = true; % Imposta a 'false' per angoli randomici
tolerance = 1e-4;
areDifferent = @(a, b, tol) abs(a - b) > tol * max(abs(a), abs(b));

%% Measuraments Variables
pr = [0,0]';
qt = - 10;
Iota = -pi/8;
Omega = pi/8;
% u = 200;
% w = 150;
% Ts = 0.001;

if use_specific_angles
    alpha = pi/10;
    phi = 0;
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

alpha_ang = rad2deg(alpha)
phi_ang = rad2deg(phi)

m1_i = tan((3*pi/2 + Iota + phi)); 
m2_o = tan((3*pi/2 + Omega + phi));
    
mt = tan(alpha);
mr = tan(phi);
% robot position

q1 = pr(2) - m1_i*pr(1);
q2 = pr(2) - m2_o*pr(1);
qp = pr(2) - mr*pr(1);

z_r = [0, -1]';
Ry_2D = [cos(phi), -sin(phi);
      sin(phi),  cos(phi)];
z_r = Ry_2D*z_r;

% computation for y1 
xc3 = (q1 - qt) / (mt - m1_i);
zc3 = mt * xc3 + qt;
ang1 = (3*pi/2 + Iota + phi)/pi*180;
y3m = norm([pr(1) - xc3; pr(2) - zc3]);
v_p3 = [xc3 - pr(1); zc3 - pr(2)];
visibile_y3 = dot(z_r, v_p3) > 0;
if ~visibile_y3
    error('Errore in visibilità y3 \n');
end

% Computation for y2
xc4 = (q2 - qt) / (mt - m2_o);
zc4 = mt * xc4 + qt;
ang2 = (Omega + phi)/pi*180;
y4m = norm([pr(1) - xc4; pr(2) - zc4]);
v_p4 = [xc4 - pr(1); zc4 - pr(2)];
visibile_y4 = dot(z_r, v_p4) > 0;

if ~visibile_y4
    error('Errore in visibilità y4 \n');
end

% valore da ottenere di h
hm = (abs(mt*pr(1) - pr(2) + qt))/ (sqrt(mt^2 + 1));

% Y computation
y3_abs = hm/abs(cos(Iota - (alpha - phi)));
y3 = hm/(cos(Iota - (alpha - phi)));
y4_abs = hm/abs(cos(Omega - (alpha - phi)));
y4 = hm/(cos(Omega - (alpha - phi)));

% h check
h_y3 = y3m*(cos(Iota - (alpha - phi)));
h_y4_abs = y4m * abs(cos(Omega - (alpha - phi)));

% h variation
% dx = u*cos(phi) + w*sin(phi);
% dz = u*sin(phi) - w*cos(phi);
% pr_new = pr + [dx*Ts, dz*Ts]'; 
% h_new = hm - u*sin((alpha - phi))*Ts - w*cos((alpha - phi))*Ts;
% hm_new = (abs(mt*pr_new(1) - pr_new(2) + qt))/ (sqrt(mt^2 + 1));

fprintf('\nValori Calcolati:\n');
fprintf('hm: %.4f\n', hm);
fprintf('y1m: %.4f\n', y3m);
fprintf('y2m: %.4f\n', y4m);

if areDifferent(hm, h_y3, tolerance) || areDifferent(h_y3, h_y4_abs, tolerance)
    fprintf('Errore in h measurament\n');
    fprintf('hm: %.4f\n', hm);
    fprintf('h_y1: %.4f\n', h_y3);
    fprintf('h_y2_abs: %.4f\n', h_y4_abs);
end

if areDifferent(y3m, y3_abs, tolerance) || areDifferent(y3_abs, y3, tolerance)
    fprintf('Errore in y1\n');
    fprintf('y1m: %.4f\n', y3m);
    fprintf('y1_abs: %.4f\n', y3_abs);
    fprintf('y1: %.4f\n', y3);
end

if areDifferent(y4m, y4_abs, tolerance) || areDifferent(y4_abs, y4, tolerance)
    fprintf('Errore in y2 \n');
    fprintf('y2m: %.4f\n', y4m);
    fprintf('y2_abs: %.4f\n', y4_abs);
    fprintf('y2: %.4f\n', y4);
end

% fprintf('Controllo velocità e valori h\n');
% fprintf('h nel nuovo punto dato dalle velocità: %.4f\n', hm_new);
% if areDifferent(hm_new, h_new, tolerance)
%     fprintf('Errore nella variazione di h \n');
%     fprintf('h dalle velocità: %.4f\n', h_new);
%     val_u = -u*sin((alpha - phi))*Ts;
%     val_w = -w*cos((alpha - phi))*Ts;
%     fprintf('velocità u: %.4f, velocità w: %.4f\n', val_u, val_w);
% end

% plot
% Intervallo di x per il plot
x_min = min([pr(1), xc3, xc4]) - 5;
x_max = max([pr(1), xc3, xc4]) + 5;
x_values = linspace(x_min, x_max, 100);

% Equazioni delle rette
y_mt = mt * x_values + qt;
y_m1g = m1_i * x_values + q1; 
y_m2l = m2_o * x_values + q2;
y_mp = mr * x_values + qp;

% plot per h
m_perp = -1/mt; 
b_perp = pr(2) - m_perp * pr(1); 
x_hm = (b_perp - qt) / (mt - m_perp);
y_hm = mt * x_hm + qt;

figure;
hold on;
grid on;
box on;

plot(x_values, y_mt, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Retta terrain (mt)');
plot(x_values, y_m1g, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Retta y1 (m1_g)');
plot(x_values, y_m2l, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Retta y2 (m2_l)');
plot(x_values, y_mp, 'y-', 'LineWidth', 1.5, 'DisplayName', 'Retta roll (mp)');

% Plot dei punti noti
plot(pr(1), pr(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Punto Pr (0,0)');
plot(xc3, zc3, 'cx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Intersezione xc1,zc1');
plot(xc4, zc4, 'yx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Intersezione xc2,zc2');
% plot(pr_new(1), pr_new(2), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Punto Pr nuovo');

plot(x_hm, y_hm, 'kd', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Punto hm sul Terreno'); % Punto hm
plot([pr(1), x_hm], [pr(2), y_hm], 'k:', 'LineWidth', 1, 'DisplayName', 'Segmento hm'); % Retta congiungente

% Aggiungi etichette e legenda
xlabel('Asse Y');
ylabel('Asse -Z');
title('Plot delle Rette nel Piano');
legend('Location', 'best');
axis equal; % Assicura che le unità sugli assi siano uguali per evitare distorsioni angolari
ylim_auto = [min([min(y_mt), min(y_m1g), min(y_m2l), qt, zc3, zc4])-5, max([max(y_mt), max(y_m1g), max(y_m2l), qt, zc3, zc4])+5];
if isfinite(ylim_auto(1)) && isfinite(ylim_auto(2))
    ylim(ylim_auto);
end
hold off;


