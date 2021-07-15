%% Kinematic.

ct0 = cos(theta0);
st0 = sin(theta0);


%% Longitudinal nondimensional stability derivatives.

% C_X derivatives
CX_u        = - Cd_u - 2*Cd_0;
CX_alpha    = - Cd_alpha + Cl_0;
CX_alphadot = - Cd_alphadot;
CX_q        = - Cd_q;
CX_deltaT   = - Cd_deltaT;
CX_deltaE   = - Cd_deltaE;

% C_Z derivatives
CZ_u        = - Cl_u - 2*Cl_0;
CZ_alpha    = - Cl_alpha - Cd_0;
CZ_alphadot = - Cl_alphadot;
CZ_q        = - Cl_q;
CZ_deltaT   = - Cl_deltaT;
CZ_deltaE   = - Cl_deltaE;

% C_M derivatives
CM_u        =  Cm_u + 2*Cm_0;
CM_alpha    =  Cm_alpha;
CM_alphadot =  Cm_alphadot;
CM_q        =  Cm_q;
CM_deltaT   =  Cm_deltaT;
CM_deltaE   =  Cm_deltaE;

