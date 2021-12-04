%.. Input Parameters
% Controls
dR = 0.0 * pi/180.0;
dP = 0.0 * pi/180.0;
dY = 0.0 * pi/180.0;
ctrl_surfaces_b = [dR;dP;dY];

% UVW
U = 185.038755;  % mach*speed_of_sound = 0.55*336.4341
V = 10.0;
W = 16.18879338;

V_b = sqrt(U^2+V^2+W^2);

% PQR
P =  0.0 * pi/180.0;
Q =  0.0 * pi/180.0;
R =  0.0 * pi/180.0;
angular_rates_b = [P;Q;R];


% UVW_dot
U_dot = 0.0;
V_dot = 0.0;
W_dot = 0.0;

% AirData
[temperature, speed_of_sound, pressure, rho] = atmosisa(1000);
alpha = atan2(W,U);
beta = asin(V/V_b);
dynamic_pressure = 1/2*rho*(V_b)^2;
mach_number = machnumber([U,V,W],speed_of_sound);

% missile parameters
 XCG         =   0.809 ;                                                 
 XREF        =   0.809 ;
 D           =   0.150 ; 

%% Calculate Coefficient Parameters

% Calculate Total Angle-of-attack
alpha_T = acos(cos(alpha)*cos(beta));

% Calculate Total Roll Angle
roll_angle_T = atan2(tan(beta),sin(alpha));

% Calculate D/2V
D_2V = D/(2*V_b);

% Compose Rotation of Matrix of body axis to aeroballistics axis
ROT_b_to_a = [1 0 0; 0 cos(roll_angle_T) -sin(roll_angle_T); 0 sin(roll_angle_T) cos(roll_angle_T)];

% Calculate effective control input dEFF
dEFF = (abs(dP)+abs(dY))/2;

% Calculate geo_param
geo_param = (XCG-XREF)/D;

% PQR[a]
angular_rates_a = ROT_b_to_a*angular_rates_b;
P_a = angular_rates_a(1);
Q_a = angular_rates_a(2);
R_a = angular_rates_a(3);

% Control surfaces in aeroballistics axis
ctrl_surfaces_a = ROT_b_to_a*ctrl_surfaces_b;
dR_a = ctrl_surfaces_a(1);
dP_a = ctrl_surfaces_a(2);
dY_a = ctrl_surfaces_a(3);

%% Compute Aero-F-Coefficients_a

% Compute CX_a
CX_0 = interp1(Tbl_MACH, Tbl_CX_0, mach_number);
CX_ALPHAT = interp1(Tbl_MACH, Tbl_CX_ALPHAT, mach_number);
CX_DEL_EFF = interp1(Tbl_MACH, Tbl_CX_DEL_EFF, mach_number);

CX_a = CX_0 + CX_ALPHAT*alpha_T + CX_DEL_EFF*(dEFF)^2;

% Compute CY_a
CY_PHIT = interp2(Tbl_ALPHAT, Tbl_MACH, Tbl_CY_PHIT, alpha_T, mach_number);
CY_DEL_Y = interp1(Tbl_MACH, Tbl_CY_DEL_Y, mach_number);

CY_a = CY_PHIT*(sin(4*roll_angle_T)) + CY_DEL_Y*dY_a;

% Compute CZ_a 
CZ_0 = interp2(Tbl_ALPHAT, Tbl_MACH, Tbl_CZ_0, alpha_T, mach_number);
CZ_PHIT = interp2(Tbl_ALPHAT, Tbl_MACH, Tbl_CZ_PHIT, alpha_T, mach_number);
CZ_DEL_P = interp1(Tbl_MACH, Tbl_CZ_DEL_P, mach_number);

CZ_a = CZ_0 + CZ_PHIT*(sin(2*roll_angle_T))^2 + CZ_DEL_P*dP_a;

%% Compurte Aer-M-Coefficient_a

% Compute CL_a
CL_ALPHAT = interp1(Tbl_MACH, Tbl_CL_ALPHAT, mach_number);
CL_P = interp1(Tbl_MACH, Tbl_CL_P, mach_number);
CL_DEL_R = interp1(Tbl_MACH, Tbl_CL_DEL_R, mach_number);

CL_a = CL_ALPHAT*(alpha_T)^2*sin(4*roll_angle_T) + CL_P*D_2V*P_a + CL_DEL_R*dR_a;

% Compute CM_a
CM_0 = interp2(Tbl_ALPHAT, Tbl_MACH, Tbl_CM_0, alpha_T, mach_number);
CM_PHIT = interp2(Tbl_ALPHAT, Tbl_MACH, Tbl_CM_PHIT, alpha_T, mach_number);
CM_Q = interp1(Tbl_MACH, Tbl_CM_Q , mach_number);
CM_DEL_P = interp1(Tbl_MACH, Tbl_CM_DEL_P, mach_number);

CM_a = CM_0 + CM_PHIT*(sin(2*roll_angle_T))^2 + CM_Q*D_2V*Q_a + CM_DEL_P*dP_a - CZ_a*geo_param;

% Compute CN_a 
CN_PHIT  = interp2(Tbl_ALPHAT, Tbl_MACH, Tbl_CN_PHIT, alpha_T, mach_number);
CN_R = interp1(Tbl_MACH, Tbl_CN_R, mach_number);
CN_DEL_Y = interp1(Tbl_MACH, Tbl_CN_DEL_Y, mach_number);

CN_a = CN_PHIT*sin(4*roll_angle_T) + CN_R*D_2V*R_a + CN_DEL_Y*dY_a + CY_a*geo_param;

aero_coefficients = [CX_a CY_a CZ_a CL_a CM_a CN_a];




