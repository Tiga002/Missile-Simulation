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
% Altitud = 0 meter
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
xd_trim

%.. Save Solution into MAT files
save('Trim_Conditions_a.mat', 'x_trim', 'u_trim', 'y_trim', 'xd_trim', 'mach_trim', 'h_trim');

%.. Log the Trim Conditions to a Text File
log_trim_conditions('Part-A-Trim-Conditions.txt', x_trim, u_trim, y_trim, mach_trim, h_trim);

% %% Part B Trimming
% % =================================================================== %
% % Operating Conidtion:
% % Mach number = [0.3 0.5 0.7 0.9 1.1];
% % Altitud = 0 meter
% % =================================================================== %
% 
% %.. Clearing reused variables and arrays
% X0 = [];
% U0 = [];
% 
% %.. Creating loop variables to store the results
% x_trim_matrix = [];
% u_trim_matrix = [];
% y_trim_matrix = [];
% xd_trim_matrix = [];
% Mach_Trim_matrix = [];
% Alt_Trim_matrix = [];
% 
% %.. Seeting trim conditions calculation
% Alt_Trim            =        0.0;                                       % Trimming Altitude of Missile
% Mach_Trim          =         [0.3 0.5 0.7 0.9 1.1];                     %Trimming Mach number of Misslile
% 
% for i=1:length(Mach_Trim);
% 
%     %.. Clearing reused variables and arrays
%     X0 = [];
%     U0 = [];
% 
%     Speed_Trim = Mach_Trim(i)*interp1(ALT,SOS_Table,Alt_Trim);          % Operating Speed calculated out of given trim conditions
% 
%     %.. Initial Guess for Trim Conditions
% 
%     X0(1)           =   Speed_Trim ;
%     X0(2)           =   0.0 ;
%     X0(3)           =   0.0 ;
%     X0(4)           =   0.0 * UNIT_DEG2RAD ;
%     X0(5)           =   0.0 * UNIT_DEG2RAD ;
%     X0(6)           =   0.0 * UNIT_DEG2RAD ;
%     X0(7)           =   0.0 * UNIT_DEG2RAD ;
%     X0(8)           =   0.0 * UNIT_DEG2RAD ;
%     X0(9)           =   0.0 * UNIT_DEG2RAD ;
%     X0(10)          =   0.0 ;
%     X0(11)          =   0.0 ;
%     X0(12)          =   0.0 ;
% 
%     U0(1)           =   0.0 ;
%     U0(2)           =   0.0 * UNIT_DEG2RAD ;
%     U0(3)           =   0.0 * UNIT_DEG2RAD ;
%     U0(4)           =   0.0 * UNIT_DEG2RAD ;
%     %---------------------------------------------------------------------%
% 
%     %.. Trim Calculation
%     [x_trim, u_trim, y_trim, xd_trim] = trim('Missile_Trim_Analysis',X0',U0')
%     xd_trim
% 
%     disp('///////////////////////////////////////////////')
%     disp('                Trim Flight Conditions                ')
%     disp('///////////////////////////////////////////////')
%     fprintf(' \n ') ;
%     fprintf(' Mach   = %3.1f m/s\n ', Mach_Trim(i)  ) ;
%     fprintf(' Altitude   = %3.1f m/s\n ', Alt_Trim  ) ;
% 
%     disp('///////////////////////////////////////////////')
%     disp('          Trim State and Trim Input            ')
%     disp('///////////////////////////////////////////////')
% 
%     fprintf(' \n ') ;
%     fprintf(' U      = %3.4f m/s\n ', x_trim(1)     ) ;
%     fprintf(' V      = %3.4f m/s\n ', x_trim(2)     ) ;
%     fprintf(' W      = %3.4f m/s\n ', x_trim(3)     ) ;
%     fprintf(' P      = %3.4f deg/s\n ', x_trim(4)* UNIT_RAD2DEG     ) ;
%     fprintf(' Q      = %3.4f deg/s\n ',x_trim(5)* UNIT_RAD2DEG      ) ;
%     fprintf(' R      = %3.4f deg/s\n ',x_trim(6) * UNIT_RAD2DEG     ) ;
%     fprintf(' PHI    = %3.4f deg/s\n ', x_trim(7) * UNIT_RAD2DEG    ) ;
%     fprintf(' THETA  = %3.4f deg/s\n ',x_trim(8) * UNIT_RAD2DEG     ) ;
%     fprintf(' PSI    = %3.4f deg/s\n ', x_trim(9)* UNIT_RAD2DEG     ) ;
%     fprintf(' ALPHA  = %3.4f deg\n ', y_trim(10)* UNIT_RAD2DEG     ) ;
%     fprintf(' BETA   = %3.4f deg\n ', y_trim(11)* UNIT_RAD2DEG     ) ;
%     fprintf(' VT     = %3.4f m/s\n ', y_trim(12)     ) ;
%     fprintf(' \n ') ;
%     fprintf(' del_T  = %3.4f N\n ',    u_trim(1) ) ;
%     fprintf(' del_r  = %3.4f deg\n ',   u_trim(2) * UNIT_RAD2DEG ) ;
%     fprintf(' del_p  = %3.4f deg\n ',   u_trim(3) * UNIT_RAD2DEG ) ;
%     fprintf(' del_y  = %3.4f deg\n ',   u_trim(4) * UNIT_RAD2DEG ) ;
% 
% 
%     % Safe and Add triming data to matrix
%     x_trim_matrix = [x_trim_matrix x_trim];
%     u_trim_matrix = [u_trim_matrix u_trim];
%     y_trim_matrix = [y_trim_matrix y_trim];
%     xd_trim_matrix = [xd_trim_matrix xd_trim];
%     Mach_Trim_matrix = [Mach_Trim_matrix Mach_Trim(i)];
%     Alt_Trim_matrix = [Alt_Trim_matrix Alt_Trim];
% 
% end
% %.. Safe all results in  the Solution File
% save( 'Trim_Solution_b.mat', 'x_trim_matrix', 'u_trim_matrix', 'y_trim_matrix', 'xd_trim_matrix', 'Mach_Trim_matrix', 'Alt_Trim_matrix' ) ;
% %---------------------------------------------------------------------%
% 
% 
% %------------- Code Section for the third Trim Question --------------%
% % Flight Condition:
% % Mach_Trim = [0.3 0.5 0.7 0.9 1.1];
% % Altitude_Trimm = 0;
% %---------------------------------------------------------------------%
% 
% %.. Clearing reused variables and arrays
% X0 = [];
% U0 = [];
% clear x_trim u_trim y_trim xd_trim;
% 
% %.. Creating loop variables to store the results
% x_trim_matrix = [];
% u_trim_matrix = [];
% y_trim_matrix = [];
% xd_trim_matrix = [];
% Mach_Trim_matrix = [];
% Alt_Trim_matrix = [];
% 
% %.. Seeting trim conditions calculation
% 
% altitude_range      =[0 1000 2000 3000 4000];                           % Trimming Altitude of Missile
% Mach_Trim           =          0.7;                                     %Trimming Mach number of Misslile
% 
% for i=1:length(altitude_range);
% 
%     %.. Clearing reused variables and arrays
%     X0 = [];
%     U0 = [];
% 
%     Alt_Trim   = altitude_range(i);
%     Speed_Trim = Mach_Trim*interp1(ALT,SOS_Table,Alt_Trim);             % Operating Speed calculated out of given trim conditions
% 
%     %.. Initial Guess for Trim Conditions
% 
%     X0(1)           =   Speed_Trim ;
%     X0(2)           =   0.0 ;
%     X0(3)           =   0.0 ;
%     X0(4)           =   0.0 * UNIT_DEG2RAD ;
%     X0(5)           =   0.0 * UNIT_DEG2RAD ;
%     X0(6)           =   0.0 * UNIT_DEG2RAD ;
%     X0(7)           =   0.0 * UNIT_DEG2RAD ;
%     X0(8)           =   0.0 * UNIT_DEG2RAD ;
%     X0(9)           =   0.0 * UNIT_DEG2RAD ;
%     X0(10)          =   0.0 ;
%     X0(11)          =   0.0 ;
%     X0(12)          =   0.0 ;
% 
%     U0(1)           =   0.0 ;
%     U0(2)           =   0.0 * UNIT_DEG2RAD ;
%     U0(3)           =   0.0 * UNIT_DEG2RAD ;
%     U0(4)           =   0.0 * UNIT_DEG2RAD ;
%     %-----------------------------------------------------------------%
% 
%     %.. Trim Calculation
% 
% 
%     [ x_trim, u_trim, y_trim, xd_trim ] = trim('Missile_Trim',X0',U0');
% 
%     xd_trim
% 
%     disp('///////////////////////////////////////////////')
%     disp('                Trim Flight Conditions                ')
%     disp('///////////////////////////////////////////////')
%     fprintf(' \n ') ;
%     fprintf(' Mach   = %3.1f m/s\n ', Mach_Trim  ) ;
%     fprintf(' Altitude   = %3.1f m/s\n ', Alt_Trim  ) ;
% 
%     disp('///////////////////////////////////////////////')
%     disp('          Trim State and Trim Input            ')
%     disp('///////////////////////////////////////////////')
% 
%     fprintf(' \n ') ;
%     fprintf(' U      = %3.4f m/s\n ', x_trim(1)     ) ;
%     fprintf(' V      = %3.4f m/s\n ', x_trim(2)     ) ;
%     fprintf(' W      = %3.4f m/s\n ', x_trim(3)     ) ;
%     fprintf(' P      = %3.4f deg/s\n ', x_trim(4)* UNIT_RAD2DEG     ) ;
%     fprintf(' Q      = %3.4f deg/s\n ',x_trim(5)* UNIT_RAD2DEG      ) ;
%     fprintf(' R      = %3.4f deg/s\n ',x_trim(6) * UNIT_RAD2DEG     ) ;
%     fprintf(' PHI    = %3.4f deg/s\n ', x_trim(7) * UNIT_RAD2DEG    ) ;
%     fprintf(' THETA  = %3.4f deg/s\n ',x_trim(8) * UNIT_RAD2DEG     ) ;
%     fprintf(' PSI    = %3.4f deg/s\n ', x_trim(9)* UNIT_RAD2DEG     ) ;
%     fprintf(' ALPHA  = %3.4f deg\n ', y_trim(10)* UNIT_RAD2DEG     ) ;
%     fprintf(' BETA   = %3.4f deg\n ', y_trim(11)* UNIT_RAD2DEG     ) ;
%     fprintf(' VT     = %3.4f m/s\n ', y_trim(12)     ) ;
%     fprintf(' \n ') ;
%     fprintf(' del_T  = %3.4f N\n ',    u_trim(1) ) ;
%     fprintf(' del_r  = %3.4f deg\n ',   u_trim(2) * UNIT_RAD2DEG ) ;
%     fprintf(' del_p  = %3.4f deg\n ',   u_trim(3) * UNIT_RAD2DEG ) ;
%     fprintf(' del_y  = %3.4f deg\n ',   u_trim(4) * UNIT_RAD2DEG ) ;
%     %-----------------------------------------------------------------%
% 
%     % Safe and Add triming data to matrix
%     x_trim_matrix = [x_trim_matrix x_trim];
%     u_trim_matrix = [u_trim_matrix u_trim];
%     y_trim_matrix = [y_trim_matrix y_trim];
%     xd_trim_matrix = [xd_trim_matrix xd_trim];
%     Mach_Trim_matrix = [Mach_Trim_matrix Mach_Trim];
%     Alt_Trim_matrix = [Alt_Trim_matrix Alt_Trim];
% 
% end
% %.. Safe all results in  the Solution File
% save( 'Trim_Solution_c.mat', 'x_trim_matrix', 'u_trim_matrix', 'y_trim_matrix', 'xd_trim_matrix', 'Mach_Trim_matrix', 'Alt_Trim_matrix' ) ;