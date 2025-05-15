clc
clear all
% terrain construction
%% Measuraments Variables
pr = [0,0]';
beta = pi/6;
theta = -pi/10;
qt = - 5;
Gamma = -pi/6;
Lambda = pi/6;

m1_g = tan((3*pi/2 + Gamma + theta)); 
m2_l = tan((3*pi/2 + Lambda + theta));
    
mt = tan(beta);
% robot position

q1 = pr(2) - m1_g*pr(1);
q2 = pr(2) - m2_l*pr(1);

% computation for y1 
xc1 = (q1 - qt) / (mt - m1_g);
yc1 = mt * xc1 + qt;
y1m = norm([pr(1) - xc1; pr(2) - yc1]);
ang1 = Gamma - (beta - theta);

% Computation for y2
xc2 = (q2 - qt) / (mt - m2_l);
yc2 = mt * xc2 + qt;
y2m = norm([pr(1) - xc2; pr(2) - yc2]);
ang2 = Lambda - (beta - theta);


% valore da ottenere di h
h = (abs(mt*pr(1) - pr(2) + qt))/ (sqrt(mt^2 + 1))
y1m
y1 = h/abs((cos(Gamma - (beta - theta))))
ang1
y2m
y2 = h/abs((cos(Lambda - (beta - theta))))
ang2
