%
% Construction of longitudinal models - notation of Stevens, Lewis, Johnson
%

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

% B matrix with elevator input

B=[Zdele;
   Mdele; 
   Xdele;
       0];

% Open-loop dynamics

A_lon=inv(E)*A;

B_lon=inv(E)*B;

% % Simulation of open-loop dynamics
% 
% C=eye(4,4); D=zeros(4,1);
% 
% syscompl=ss(A_lon,B_lon,C,D);
% 
% figure(1)
% step(syscompl),grid



% Short period approximation

A_lon_sp=inv(E(1:2,1:2))*A(1:2,1:2);

B_lon_sp=inv(E(1:2,1:2))*B(1:2);


% Phugoid approximation

A_11=A(1:2,1:2);
A_12=A(1:2,3:4);
A_21=A(3:4,1:2);
A_22=A(3:4,3:4);

A_lon_ph=A_22-A_21*inv(A_11)*A_12;

% Eigenvalue plots

eig_lon=eig(A_lon);
eig_lon_sp=eig(A_lon_sp);
eig_lon_ph=eig(A_lon_ph);

figure(1)
plot(eig_lon,'x'),grid, hold
plot(eig_lon_sp,'xr')
plot(eig_lon_ph,'xr')

% Transfer functions

Gqdele=-tf(ss(A_lon,B_lon,[0 1 0 0],0));

Gqdele_sp=-tf(ss(A_lon_sp,B_lon_sp,[0 1],0));

% Bode plots

figure(4)
bode(Gqdele,Gqdele_sp),grid

% Pole/zero maps

figure(5)
pzmap(Gqdele,Gqdele_sp),grid

% Step responses

figure(6)
t=(0:.1:60);
step(Gqdele,Gqdele_sp,t),grid

