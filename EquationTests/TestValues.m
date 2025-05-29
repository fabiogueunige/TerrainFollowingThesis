clear; close all; clc;

% Flag per usare angoli specifici o generati randomicamente
use_specific_angles = true; % Imposta a 'false' per angoli randomici
tolerance = 1e-4;
areDifferent = @(a, b, tol) abs(a - b) > tol * max(abs(a), abs(b));

%% Measuraments Variables
pr = [0,0]';
qt = - 10;
Gamma = -pi/8;
Lambda = pi/8;

if use_specific_angles
    beta = pi/4;
    theta = 0;
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

beta_ang = rad2deg(beta)
theat_ang = rad2deg(theta)

m1_g = tan((3*pi/2 + Gamma + theta)); 
m2_l = tan((3*pi/2 + Lambda + theta));
    
mt = tan(beta);
mp = tan(theta);
% robot position

q1 = pr(2) - m1_g*pr(1);
q2 = pr(2) - m2_l*pr(1);
qp = pr(2) - mp*pr(1);

z_r = [0, -1]';
Ry_2D = [cos(theta), -sin(theta);
      sin(theta),  cos(theta)];
z_r = Ry_2D*z_r;

% computation for y1 
xc1 = (q1 - qt) / (mt - m1_g);
zc1 = mt * xc1 + qt;
ang1 = (3*pi/2 + Gamma + theta)/pi*180;
y1m = norm([pr(1) - xc1; pr(2) - zc1]);
v_p1 = [xc1 - pr(1); zc1 - pr(2)];
visibile_y1 = dot(z_r, v_p1) > 0;
if ~visibile_y1
    fprintf('Errore in visibilità y1 \n');
    y1m = 500000;
end

% Computation for y2
xc2 = (q2 - qt) / (mt - m2_l);
zc2 = mt * xc2 + qt;
ang2 = (Lambda + theta)/pi*180;
y2m = norm([pr(1) - xc2; pr(2) - zc2]);
v_p2 = [xc2 - pr(1); zc2 - pr(2)];
visibile_y2 = dot(z_r, v_p2) > 0;

if ~visibile_y2
    fprintf('Errore in visibilità y2 \n');
    y2m = 500000;
end

% valore da ottenere di h
hm = (abs(mt*pr(1) - pr(2) + qt))/ (sqrt(mt^2 + 1));

% Y computation
y1_abs = hm/abs(cos(Gamma - (beta - theta)));
y1 = hm/(cos(Gamma - (beta - theta)));
y2_abs = hm/abs(cos(Lambda - (beta - theta)));
y2 = hm/(cos(Lambda - (beta - theta)));

% h check
h_y1 = y1m*(cos(Gamma - (beta - theta)));
h_y2_abs = y2m * abs(cos(Lambda - (beta - theta)));

fprintf('\nValori Calcolati:\n');
fprintf('hm: %.4f\n', hm);
fprintf('y1m: %.4f\n', y1m);
fprintf('y2m: %.4f\n', y2m);

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

% plot
% Intervallo di x per il plot
x_min = min([pr(1), xc1, xc2]) - 5;
x_max = max([pr(1), xc1, xc2]) + 5;
x_values = linspace(x_min, x_max, 100);

% Equazioni delle rette
y_mt = mt * x_values + qt;
y_m1g = m1_g * x_values + q1; 
y_m2l = m2_l * x_values + q2;
y_mp = mp * x_values + qp;

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
plot(x_values, y_mp, 'p-', 'LineWidth', 1.5, 'DisplayName', 'Retta pitch (mp)');

% Plot dei punti noti
plot(pr(1), pr(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Punto Pr (0,0)');
plot(xc1, zc1, 'cx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Intersezione xc1,zc1');
plot(xc2, zc2, 'yx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Intersezione xc2,zc2');

plot(x_hm, y_hm, 'kd', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Punto hm sul Terreno'); % Punto hm
plot([pr(1), x_hm], [pr(2), y_hm], 'k:', 'LineWidth', 1, 'DisplayName', 'Segmento hm'); % Retta congiungente

% Aggiungi etichette e legenda
xlabel('Asse X');
ylabel('Asse Y');
title('Plot delle Rette nel Piano');
legend('Location', 'best');
axis equal; % Assicura che le unità sugli assi siano uguali per evitare distorsioni angolari
ylim_auto = [min([min(y_mt), min(y_m1g), min(y_m2l), qt, zc1, zc2])-5, max([max(y_mt), max(y_m1g), max(y_m2l), qt, zc1, zc2])+5];
if isfinite(ylim_auto(1)) && isfinite(ylim_auto(2))
    ylim(ylim_auto);
end
hold off;


