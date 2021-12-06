%% Loading of the linearized Model

load Missile_Linear_Model.mat;

%% Stability Analysis of Longitudinal Dynamics

%.. Computing the eigenvalues
eigenvalues_longitudinal_dynamics = eig(A_lon)
longitudinal_sys = ss(A_lon, B_lon, C_lon, D_lon);
figure(1)
pzmap(longitudinal_sys)
title('Pole Locations of Longitudinal Dynamics')
xlabel('Real Axis')
ylabel('Imaginary Axis')
grid on

% Compute the damping ratio and natrual frequency
damp(longitudinal_sys)

%% Stability Analysis of Lateral Dynamics

%computing of eigenvalues
eigenvalues_lateral_dynamics = eig(A_lat)
lateral_sys = ss(A_lat, B_lat, C_lat, D_lat)
figure(2)
pzmap(lateral_sys)
title('Pole Locations of Lateral Dynamics')
xlabel('Real Axis')
ylabel('Imaginary Axis')
grid on

% Compute the damping ratio and natrual frequency
damp(lateral_sys)

