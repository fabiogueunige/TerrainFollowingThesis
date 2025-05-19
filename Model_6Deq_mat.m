% Equilibrium point and system specification
m = 11.5; % massa totale [kg]
I = diag([0.21, 0.245, 0.245]);
p = 10; % polo del sistema (non specificato nei file)
wn = 0.2; % frequenza naturale (ipotizzata)
damp = 0.6; % smorzamento (ipotizzato)

% Condizioni iniziali e saturazioni
speed0 = [0.1, 0, 0, 0, 0, 0]'; % velocità iniziale (surge=0.1 m/s)
sat_surge = 15; sat_dep = 15; sat_yaw = 15;
sat_tr = 2000;

% [surge, sway, heave, roll, pitch, yaw]
% Added mass
tau_a = [27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 

% Linear damping
tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)

% Quadratic damping
tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)

% Virtual mass
mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

% Dissipative forces (con v0 = 0.1 solo nel surge)
dv = -tau_r - 2 * tau_d .* abs(speed0);