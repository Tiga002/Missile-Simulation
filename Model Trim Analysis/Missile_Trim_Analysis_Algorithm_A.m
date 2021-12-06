%/////////////////////////////////////////////////////////////////////////%
%                                                                         %
%   - Name :Missile_Trim_Analysis_Algorithm.m                                          %
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

%% Part A Trimming
% =================================================================== %
% Operating Conidtion:
% Mach number = 0.7
% Altitude = 0 meter
% =================================================================== %

%.. Setting Operating Codintions for Part A
mach_trim = 0.7;
h_trim = 0.0;
V_speed_trim = mach_trim * interp1(Tbl_ALT,Tbl_SOS, h_trim);

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
[ x_trim, u_trim, y_trim, xd_trim ] = trim('Missile_Trim_Analysis',X0',U0');

%.. Check the State Derivatives to verify the optimization
%xd_trim

%.. Save Solution into MAT files
save('Trim_Conditions_a.mat', 'x_trim', 'u_trim', 'y_trim', 'xd_trim', 'mach_trim', 'h_trim');

%.. Log the Trim Conditions to a Text File
log_trim_conditions('Part-A-Trim-Conditions.txt', x_trim, u_trim, y_trim, mach_trim, h_trim);
