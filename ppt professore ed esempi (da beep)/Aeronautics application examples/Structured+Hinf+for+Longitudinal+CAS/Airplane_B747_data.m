% Airplane B747 Data
%
% M = 0.25
% Sea Level
%
% set geometric, inertial and linearized aerodynamic
% characteristics of airplane A in configuration 1
%
% SI Unit
%
% Last modified 06.04.2004 by A.C.
% ---------------------------------
% DIA - Politecnico di Milano, 2004

g = 9.80665;
theta0 = 0.0*pi/180;

% Geometric Data  - N.B. we do NOT need to change US!!!
mass            = 636600 * 0.45359237;          % lbs --> kg
wing_surface    = 5500   * 0.3048*0.3048;       % ft^2 --> m^2
chord           = 27.31  * 0.3048;              % ft --> m
span            = 195.68 * 0.3048;              % ft --> m
Ixx             = 18.20E+06 * 14.5939*0.3048^2; % slug ft^2 --> kg m^2
Iyy             = 33.10E+06 * 14.5939*0.3048^2; % slug ft^2 --> kg m^2
Izz             = 49.70E+06 * 14.5939*0.3048^2; % slug ft^2 --> kg m^2
Ixz             =  0.97E+06 * 14.5939*0.3048^2; % slug ft^2 --> kg m^2
sweep_le        = 20 * pi/180;                  % rad

% Flight condition and atmospheric parameters
% Default parameters
height          = 00000.0 * 0.3048;             % ft --> m
[pres,density,temper] = isatm(height);          % S.I. system 
Mach            = 0.25;
% Example of modified parameters
% height            = 10000.0 * 0.3048;             % ft --> m
% [pres,density,temper] = isatm(height);          % S.I. system 
% Mach            = 0.4;

U0              = sqrt(1.4*287.05*temper)*Mach ;% m/sec
dyn_press       = 0.5*density*U0^2;             % Pa 
w0 = 0;
q0 = 0;

% Longitudinal Aerodynamic Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Lift Coefficients
Cl_0        = 1.11;
Cl_alpha    = 5.7;
Cl_alphadot = 6.7;
Cl_q        = 5.4;
Cl_deltaT   = 0.0;
Cl_deltaE   = 0.338;
Cl_u        = -0.81*Mach; % Cl_u = Mach*Cl_M;
    % Pitching Moment Coefficients
Cm_0        = 0.0;
Cm_alpha    = -1.26;
Cm_alphadot = -3.2;
Cm_q        = -20.8;
Cm_deltaT   = 0.0;
Cm_deltaE   = -1.34;
Cm_u        = 0.27*Mach; % Cm_u = Mach*Cm_M
    % Drag Coefficients
Cd_0        = 0.102;
Cd_alpha    = 0.66;
Cd_alphadot = 0.0;
Cd_q        = 0.0;
Cd_deltaT   =-1.0;
Cd_deltaE   = 0.0;
Cd_u        = 0.0*Mach; % Cd_u = Mach*Cd_M;

% Lateral Aerodynamic Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Side Force Coefficients
CY_beta     = -0.96;
CY_betadot  = 0.00;
CY_p        = 0.0;
CY_r        = 0.0;
CY_deltaR   = 0.175;
CY_deltaA   = 0.0;
    % Rolling Moment Coefficients
CL_beta     = -0.221;
CL_betadot  = 0.00;
CL_p        = -0.45;
CL_r        = 0.101;
CL_deltaR   = 0.007;
CL_deltaA   = 0.0461;
    % Yawing Moment COefficients
CN_beta     = 0.150;
CN_betadot  = 0.000;
CN_p        = -0.121;
CN_r        = -0.3;
CN_deltaR   = -0.109;
CN_deltaA   = 0.0064;
