% D component of gravity
gD=9.81;

% Redefinition of trim values  

alpe=0;
game=0;
VTe=U0;

% Redefinition of parameters for denormalisation of dimensionless
% derivatives

qbar=dyn_press;

S=wing_surface; 

m=mass;

b=span; 

cbar=chord;

alpT=0;

% Computation of dimensional aerodynamic derivatives

Xalp=qbar*S/m*(Cl_0-Cd_alpha);

XV=-qbar*S/(m*VTe)*(2*Cd_0+Cd_u);
XTV=0;

Xdele=-qbar*S/m*Cd_deltaE;

Xdelt=-qbar*S/m*Cd_deltaE;

Zalp=-qbar*S/m*(Cd_0+Cl_alpha);

Zalpd=-qbar*S*cbar/(2*m*VTe)*Cl_alphadot;

ZV=-qbar*S/(m*VTe)*(2*Cl_0+Cl_u);

Zq=-qbar*S*cbar/(2*m*VTe)*Cl_q;

Zdele=-qbar*S/m*Cl_deltaE;

Malp=qbar*S*cbar/Iyy*Cm_alpha;
MTalp=0;

Malpd=qbar*S*cbar/Iyy*(cbar/2/VTe)*Cm_alphadot;

Mq=qbar*S*cbar/Iyy*(cbar/2/VTe)*Cm_q;

MV=qbar*S*cbar/Iyy/VTe*(2*Cm_0+Cm_u);
MTV=0;

Mdele=qbar*S*cbar/Iyy*Cm_deltaE;

% Construction of linearised longitudinal model

E=[VTe-Zalpd, 0, 0, 0; 
   -Malpd,    1, 0, 0; 
   0,         0, 1, 0; 
   0,         0, 0, 1];

A=[Zalp,        VTe+Zq, ZV-XTV*sin(alpe+alpT), -gD*sin(game); 
   Malp+MTalp,      Mq,                MV+MTV,             0; 
   Xalp,             0, XV+XTV*sin(alpe+alpT), -gD*cos(game); 
   0,                1,                     0,             0];

% B=[Zdele, -Xdelt*sin(alpe+alpT);
%    Mdele,                 Mdelt; 
%    Xdele,  Xdelt*cos(alpe+alpT);
%        0,                     0];

B=[Zdele;
   Mdele; 
   Xdele;
       0];

% Open-loop dynamics

A_lon=inv(E)*A;

B_lon=inv(E)*B;

% Short period approximation

A_lon_sp=inv(E(1:2,1:2))*A(1:2,1:2);

B_lon_sp=inv(E(1:2,1:2))*B(1:2);


Gqdele=-tf(ss(A_lon,B_lon,[0 1 0 0],0));

Gqdele_sp=-tf(ss(A_lon_sp,B_lon_sp,[0 1],0));

%
% SAS gain closure
%

KQ=0.528;

Gqdele_sp_sas=Gqdele_sp/(1+KQ*Gqdele_sp);

Gqdele_sp_sas=minreal(Gqdele_sp_sas);

figure,step(Gqdele_sp,Gqdele_sp_sas)

figure, bode(Gqdele_sp,Gqdele_sp_sas)

% effect of zero
figure,step(Gqdele_sp_sas,tf(0.2763,[1 1.305 0.8692]))

%
% CAS design: root loci with I and PI
%

figure
rlocus(tf(1,[1 0])*Gqdele_sp_sas);

figure
rlocus(tf([5 1],[1 0])*Gqdele_sp_sas);

%
% Loop transfer functions (no PI zero and muR=1)
%

Lqdele_sp_cas_I=tf(1,[1 0])*Gqdele_sp_sas;

figure,margin(Lqdele_sp_cas_I),grid

%
% Loop transfer functions (PI zero and tuned muR)
%

Lqdele_sp_cas_PI=tf(3*[2 1],[1 0])*Gqdele_sp_sas;

figure,
margin(Lqdele_sp_cas_PI)



