clc; clear; close all;

%% Parametri della circonferenza
R = 10;               % Raggio della circonferenza
center = [5, 5];      % Centro della circonferenza (x_c, y_c)

%% Posizione attuale del robot
x_robot = 12;         % Posizione X del robot
y_robot = 15;         % Posizione Y del robot
pos_robot = [x_robot, y_robot];

%% Calcolo del punto più vicino sulla circonferenza
vector_to_robot = pos_robot - center;  % Vettore dal centro al robot
distance_to_center = norm(vector_to_robot);  % Distanza dal centro al robot
factor = R / distance_to_center;  % Fattore di normalizzazione
pos_nearest = center + factor * vector_to_robot;  % Punto più vicino sulla circonferenza

%% Calcolo del Cross Track Error (CTE)
cte_vector = pos_robot - pos_nearest;  % Vettore di errore
cte = norm(cte_vector);  % Normale = distanza

%% Direzione Line-of-Sight (LOS)
% La direzione del controllore LOS è dalla posizione del robot al centro della circonferenza
los_vector = center - pos_robot;  % Direzione verso il centro della circonferenza
los_direction = los_vector / norm(los_vector);  % Normalizzazione

%% Calcolare l'angolo psi (angolo di orientamento desiderato)
% Angolo tra la posizione del robot e la direzione LOS
theta_los = atan2(los_direction(2), los_direction(1));

%% Visualizzazione
figure; hold on; grid on; axis equal;
theta = linspace(0, 2*pi, 100);  % Parametro angolare per la circonferenza
x_circle = center(1) + R * cos(theta);  % X della circonferenza
y_circle = center(2) + R * sin(theta);  % Y della circonferenza
plot(x_circle, y_circle, 'b', 'LineWidth', 2);  % Traiettoria circolare

plot(x_robot, y_robot, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % Robot
plot(pos_nearest(1), pos_nearest(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % Punto più vicino
quiver(x_robot, y_robot, cte_vector(1), cte_vector(2), 0, 'r', 'LineWidth', 2);  % Errore CTE
quiver(x_robot, y_robot, los_direction(1), los_direction(2), 0.5, 'g', 'LineWidth', 2);  % Direzione LOS

legend('Circonferenza', 'Robot', 'Punto più vicino', 'Errore CTE', 'Direzione LOS');
title(sprintf('CTE = %.2f, Psi_{LOS} = %.2f°', cte, rad2deg(theta_los)));
