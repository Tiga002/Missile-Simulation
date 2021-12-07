
%/////////////////////////////////////////////////////////////////////////%
%                                                                         %
%   - Name :Missile_Trim_Analysis_Algorithm_B.m                                          %
%                                                                         %
%                                                                         %
%/////////////////////////////////////////////////////////////////////////%

%% Matlab Initialize

clc;
close all;
clear all ;

%.. Load Sim Parameters and Missile Data
Sim_Parameters ;
Missile_Data;

%% Operating Codintions

global      h_trim  V_speed_trim    mach_trim

h_trim = 0.0;
V_speed_trim = 0.0;
mach_trim = 0.0;

%% Constraints

% Turn rate Constraint from Trim Analysis
global psi_dot_trim
psi_dot_trim =  0.0 * UNIT_DEG2RAD ;

% Pull-up Constraint from Trim Analysis
global theta_dot_trim
theta_dot_trim = 0.0 * UNIT_DEG2RAD;

% Coordinate Turn Constraint [CTC] from Trim Analysis
global  G_turn  CTC
CTC = 0.0;
G_turn = psi_dot_trim * V_speed_trim/ UNIT_GRAV;

% Rate of Climb Constraint [ROC] from Trim Analysis
global gamma_trim   ROC
ROC = 0.0;
gamma_trim = 0.0 * UNIT_DEG2RAD;

% Speed Constraints [SC] from Trim Analysis
global SC
SC = 0.0;

%% Find names and ordering of states from Simulink model

[ sizes, ss, names] = Missile_Trim_Analysis
%
% disp('///////////////////////////////////////////////')
% disp('          Check order of state variables')
% disp('///////////////////////////////////////////////')
% names{:}

%% Part B Trimming
% =================================================================== %
% Operating Conidtion:
% Mach number = [0.3 0.4 0.6 0.7]
% Altitude = 0 meter
% =================================================================== %

%.. Clear reused variables and arrays
X0 = [];
U0 = [];

%.. Creating loop variables to store the results
x_trim_matrix = [];
u_trim_matrix = [];
y_trim_matrix = [];
xd_trim_matrix = [];
mach_trim_matrix = [];
h_trim_matrix = [];

%.. Setting Operating Codintions for Part B
mach_trim = [0.3 0.4 0.6 0.7];
h_trim = 0.0;
fileID = fopen('Part-B-Trim-Conditions.txt', 'w');
for i=1:length(mach_trim)

    %.. Clear reused variables and arrays
    X0 = [];
    U0 = [];

    V_speed_trim = mach_trim(i) * interp1(Tbl_ALT,Tbl_SOS, h_trim);

    %.. Initial Guess of Trim States and Inputs

    X0(1)           =   V_speed_trim ;                                          % U                         (m/s)
    X0(2)           =   0.0 ;                                                   % V                         (m/s)
    X0(3)           =   0.0 ;                                                   % W                         (m/s)
    X0(4)           =   0.0 * UNIT_DEG2RAD ;                                    % P                         (rad/s)
    X0(5)           =   0.0 * UNIT_DEG2RAD ;                                    % Q                         (rad/s)
    X0(6)           =   0.0 * UNIT_DEG2RAD ;                                    % R                         (rad/s)
    X0(7)           =   0.0 * UNIT_DEG2RAD ;                                    % PHI                       (rad)
    X0(8)           =   0.0 * UNIT_DEG2RAD ;                                    % THETA                     (rad)
    X0(9)           =   0.0 * UNIT_DEG2RAD ;                                    % PSI                       (rad)
    X0(10)          =   CTC ;                                                   % CTC
    X0(11)          =   ROC ;                                                   % ROC
    X0(12)          =   SC ;                                                    % SC

    U0(1)           =   0.0 ;                                                   % THRUST                    (HP)
    U0(2)           =   0.0 * UNIT_DEG2RAD ;                                    % Elevator                  (rad)
    U0(3)           =   0.0 * UNIT_DEG2RAD ;                                    % Aileron                   (rad)
    U0(4)           =   0.0 * UNIT_DEG2RAD ;                                    % Rudder                    (rad)

    %.. Find the Trim Point of the Nonlinear Dynamic System
    [x_trim, u_trim, y_trim, xd_trim] = trim('Missile_Trim_Analysis',X0',U0');
    
    %.. Check the State Derivatives to verify the optimization
    xd_trim

    %.. Log the Trim Conditions to a Text File
    log_trim_conditions_iterative(fileID, x_trim, u_trim, y_trim, mach_trim(i), h_trim);

    % Safe and Add triming data to matrix
    x_trim_matrix = [x_trim_matrix x_trim];
    u_trim_matrix = [u_trim_matrix u_trim];
    y_trim_matrix = [y_trim_matrix y_trim];
    xd_trim_matrix = [xd_trim_matrix xd_trim];
    mach_trim_matrix = [mach_trim_matrix mach_trim(i)];
    h_trim_matrix = [h_trim_matrix h_trim];

end
%.. Save Solution into MAT files
save('Trim_Conditions_b.mat', 'x_trim_matrix', 'u_trim_matrix', 'y_trim_matrix', 'xd_trim_matrix', 'mach_trim_matrix', 'h_trim_matrix')

%% Analysis
tiledlayout(3,1)
% Plot Trim Angle-of-Attack against Mach Number
trim_angle_of_attack_arr = y_trim_matrix(10,:);
nexttile
plot(mach_trim_matrix, trim_angle_of_attack_arr, '-r');
title('Trim Angle-of-Attack against Mach Number');
xlabel('Mach Number');
ylabel('Angle-of-attack (rad)');

% Plot Trim pitch control fin deflection against Mach Number
trim_elevator_deflection_arr = u_trim_matrix(2,:);
nexttile
plot(mach_trim_matrix, trim_elevator_deflection_arr, '-g');
title('Trim Pitch Control Fin Deflection against Mach Number');
xlabel('Mach Number');
ylabel(' Pitch control fin deflection (rad)');

% Plot Trim Thrust against Mach Number
trim_thrust_arr = u_trim_matrix(1,:);
nexttile
plot(mach_trim_matrix,trim_thrust_arr, '-b');
title('Trim Thrust against Mach Number');
xlabel('Mach Number');
ylabel('Trim thrust (N)');



