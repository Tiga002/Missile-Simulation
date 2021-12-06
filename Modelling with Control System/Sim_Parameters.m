%/////////////////////////////////////////////////////////////////////////%
%                                                                         %
%   - Name : Sim_Parameters.m                                             %
%   - Simulation Parameters are defined.                                  %
%                                                                         %
%                               - Created by C. H. Lee, 12/10/2018        %
%                                                                         %
%/////////////////////////////////////////////////////////////////////////%

%.. Global Variables

    global  UNIT_RAD2DEG    UNIT_DEG2RAD    UNIT_GRAV          
    global  Init_Pos        Init_Vel        Init_Euler      Init_Rate

%.. Load Trim Conditions Found 

    load Trim_Conditions_a.mat ;

%.. Unit Conversion 

    UNIT_RAD2DEG        =       180 / pi ;                              	% Radian to Degree
    UNIT_DEG2RAD        =       1 / UNIT_RAD2DEG ;                       	% Degree to Radian
    UNIT_GRAV           =       9.81 ;                                      % Gravity
    
%.. Setting up Simulation Parameters

    Step_Size           =       0.0001 ;                                     % Step_Size for Numerical Integration
    windOn              =       0;
    
%.. Initial Conditions (you can change these values as you want)

    Init_Pos            =       [ 0.0, 0.0, 0.0 ] ;                         % Xe, Ye, Ze            (m)
    Init_Vel            =       x_trim(1:3) ; 
    Init_Euler          =       x_trim(7:9)  ; 
    Init_Rate           =       x_trim(4:6)  ; 
    