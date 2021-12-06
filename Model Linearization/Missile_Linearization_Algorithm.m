%/////////////////////////////////////////////////////////////////////////%
%                                                                         %
%   - Name : Missile_Linearization_Algorithm.m                                           %
%                                                                         %
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

%% Load Trim Solution Found

load Trim_Conditions_a.mat ;

%.. Linear Model Calculation

[sizes, x0, names] = Missile_Linearization

%% Initial/Trim Conditions 
X0(1) = 0.0;                                                                 % Xe                (m)
X0(2) = 0.0;                                                                 % Ye                (m)
X0(3) = -h_trim;                                                             % Ze                (m)
X0(4:6) = x_trim(4:6);                                                       % P, Q, R           (rad/s)
X0(7:9) = x_trim(7:9);                                                       % Phi, Theta, Psi   (rad)
X0(10:12) = x_trim(1:3);                                                     % U, V, W           (m/s)

U0 = u_trim;                                                                 % throttle, elevator, aileron, rudder

%---------------------------------------------------------------------%

%% Linearization
[A, B, C, D]    =  linmod('Missile_Linearization',X0',U0');

%% Longitudinal Dynamics
%/////////////////////////////////////////////////////////////////%
%   - X = [ U, W, Q, THETA ]'                                     %
%   - U = [ elevator_trim ]                                      %
%/////////////////////////////////////////////////////////////////%

%.. Initialize the longitudinal motion state-space matrices
A_lon = zeros( 4, 4 ) ;
B_lon = zeros( 4, 1 ) ;
C_lon = zeros( 4, 4 ) ;
D_lon = zeros( 4, 1 ) ;

Idx_U = 10;
Idx_W = 12;
Idx_Q = 5;
Idx_THETA = 8;


A_lon(1,:) = [ A(Idx_U,Idx_U), A(Idx_U,Idx_W), A(Idx_U,Idx_Q), A(Idx_U,Idx_THETA) ] ;
A_lon(2,:) = [ A(Idx_W,Idx_U), A(Idx_W,Idx_W), A(Idx_W,Idx_Q), A(Idx_W,Idx_THETA) ] ;
A_lon(3,:) = [ A(Idx_Q,Idx_U), A(Idx_Q,Idx_W), A(Idx_Q,Idx_Q), A(Idx_Q,Idx_THETA) ] ;
A_lon(4,:) = [ A(Idx_THETA,Idx_U), A(Idx_THETA,Idx_W), A(Idx_THETA,Idx_Q), A(Idx_THETA,Idx_THETA) ] ;

B_lon(:,1) = [ B(Idx_U,2), B(Idx_W,2), B(Idx_Q,2), B(Idx_THETA,2) ]' ;

C_lon(1,:) = [ C(Idx_U,Idx_U), C(Idx_U,Idx_W), C(Idx_U,Idx_Q), C(Idx_U,Idx_THETA) ] ;
C_lon(2,:) = [ C(Idx_W,Idx_U), C(Idx_W,Idx_W), C(Idx_W,Idx_Q), C(Idx_W,Idx_THETA) ] ;
C_lon(3,:) = [ C(Idx_Q,Idx_U), C(Idx_Q,Idx_W), C(Idx_Q,Idx_Q), C(Idx_Q,Idx_THETA) ] ;
C_lon(4,:) = [ C(Idx_THETA,Idx_U), C(Idx_THETA,Idx_W), C(Idx_THETA,Idx_Q), C(Idx_THETA,Idx_THETA) ] ;

D_long(:,1) = [ D(Idx_U,2), D(Idx_W,2), D(Idx_Q,2), D(Idx_THETA,2) ]' ;

A_lon
B_lon
C_lon
D_lon

%% Lateral Dynamics
%/////////////////////////////////////////////////////////////////%
%   - X = [ V, P, R, PHI ]'                                       %
%   - U = [ aileron_trim, rudder_trim ]'                                       %
%/////////////////////////////////////////////////////////////////%

%.. Initialize the lateral motion state-space matrices
A_lat = zeros( 4, 4 ) ;
B_lat = zeros( 4, 2 ) ;
C_lat = zeros( 4, 4 ) ;
D_lat = zeros( 4, 2 ) ;

Idx_V = 11;
Idx_P = 4;
Idx_R = 6;
Idx_PHI = 7;

A_lat(1,:)      =       [ A(  Idx_V,Idx_V), A(  Idx_V,Idx_P), A(  Idx_V,Idx_R), A(  Idx_V,Idx_PHI) ] ;
A_lat(2,:)      =       [ A(  Idx_P,Idx_V), A(  Idx_P,Idx_P), A(  Idx_P,Idx_R), A(  Idx_P,Idx_PHI) ] ;
A_lat(3,:)      =       [ A(  Idx_R,Idx_V), A(  Idx_R,Idx_P), A(  Idx_R,Idx_R), A(  Idx_R,Idx_PHI) ] ;
A_lat(4,:)      =       [ A(Idx_PHI,Idx_V), A(Idx_PHI,Idx_P), A(Idx_PHI,Idx_R), A(Idx_PHI,Idx_PHI) ] ;

B_lat(:,1)      =       [ B(Idx_V,3), B( Idx_P,3), B(Idx_R,3), B(Idx_PHI,3) ]' ;
B_lat(:,2)      =       [ B(Idx_V,4), B( Idx_P,4), B(Idx_R,4), B(Idx_PHI,4) ]' ;

C_lat(1,:)      =       [ C(  Idx_V,Idx_V), C(  Idx_V,Idx_P), C(  Idx_V,Idx_R), C(  Idx_V,Idx_PHI) ] ;
C_lat(2,:)      =       [ C(  Idx_P,Idx_V), C(  Idx_P,Idx_P), C(  Idx_P,Idx_R), C(  Idx_P,Idx_PHI) ] ;
C_lat(3,:)      =       [ C(  Idx_R,Idx_V), C(  Idx_R,Idx_P), C(  Idx_R,Idx_R), C(  Idx_R,Idx_PHI) ] ;
C_lat(4,:)      =       [ C(Idx_PHI,Idx_V), C(Idx_PHI,Idx_P), C(Idx_PHI,Idx_R), C(Idx_PHI,Idx_PHI) ] ;

D_lat(:,1)      =       [ D(Idx_V,3), D( Idx_P,3), D(Idx_R,3), D(Idx_PHI,3) ]' ;
D_lat(:,2)      =       [ D(Idx_V,4), D( Idx_P,4), D(Idx_R,4), D(Idx_PHI,4) ]' ;

A_lat
B_lat
C_lat
D_lat


save( 'Missile_Linear_Model.mat', 'A_lon', 'B_lon', 'C_lon', 'D_lon', 'A_lat', 'B_lat', 'C_lat', 'D_lat') ;
