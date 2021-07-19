clear, clc, close all 

%% Creation of matrices

Y_v = -0.264;
Y_p= 0; 
L_v = -7.349;
L_p= 0;
Y_delta = 9.568;
L_delta = 1079.339;
g=9.8;

Ts=0.004;

A = [Y_v  Y_p  g;
     L_v  L_p  0;
      0    1   0];
 
B = [Y_delta L_delta 0]';

C = [0 1 0;
     0 0 1];
D = [0 0]';

G = ss(A,B,C,D);
G = c2d(G,Ts);
G.InputName = 'delta_lat';
G.OutputName = {'p';'phi'};

z = tf('z',Ts);
s = tf('s');

%% controllore R: contiene sia Rp che Rphi
b = realp('b',1);
c1 = realp('c1',1);
c2 = realp('c2',1);
d1 = realp('d1',1);
d2 = realp('d2',1);
D_phi = realp('D_phi',1);

Ap=[1 0;0 0];
Bp=[b -b*D_phi; 0 0.5];
Cp=[c1 c2];
Dp=[d1 d2*D_phi];

R = ss(Ap,Bp,Cp,Dp,Ts);
R.InputName={'e_phi';'p'};
R.OutputName={'delta_lat'};
%% creazione del modello per systune
% serve a TF da phi0 a phi 

S = sumblk('%e_phi=phi0-%phi',R.InputName(1),G.OutputName(2));

% F0 = connect(G,R,S,S.InputName(1),G.OutputName(2));
F0 = connect(G,R,S,S.InputName(1),{'phi'});
%% systune 

% req1 è per lo step da phi0 a phi
% req1 = TuningGoal.StepTracking(F0.InputName,F0.OutputName,1/10,0.91);
omega_min=10;
csi=0.9;
F2=omega_min^2/(s^2+2*csi*omega_min*s+omega_min^2); %reference tf
req1 = TuningGoal.Transient(F0.InputName,F0.OutputName,F2);

SoftReqs = [req1]; % soft requirements sono obiettivi
HardReqs = []; % hard requirements sono vincoli
opt = systuneOptions('Display','final','RandomStart',5);
[F,fSoft,gHard,info] = systune(F0,SoftReqs,HardReqs,opt);
%% plots from phi0 to phi

figure(1) %F vs F2
opt_bodeF = bodeoptions;
opt_bodeF.Title.String = '\textbf{Bode Diagram}';
opt_bodeF.Title.Interpreter = 'latex';
opt_bodeF.Title.FontSize = 12;
opt_bodeF.XLabel.String = '\textbf{Frequency}';
opt_bodeF.XLabel.Interpreter = 'latex';
opt_bodeF.XLabel.FontSize = 10;
opt_bodeF.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_bodeF.YLabel.Interpreter = 'latex';
opt_bodeF.YLabel.FontSize = 10;
opt_bodeF.Grid = 'on';

bode(F,F2,opt_bodeF)
set(findall(gcf,'type','line'),'linewidth',1.1)
legend('F','F2')

figure(2) % step response comparison of F and F2
stepplot(F,F2),legend('F','F2'), grid on
title('\textbf{Step Response}','Interpreter','latex')
xlabel('Time','Interpreter','latex')
ylabel('Amplitude','Interpreter','latex')
%% controllo sul control effort
% figure(3), bode(F(2),tf(0.03)), grid on , legend('delta_lat','0,03')
% figure(4), step(F(2)), grid on

%% Evaluation of damping factor
L=F/(1-F);
[~,Pm]=margin(L);
csi_1=sin(deg2rad(50)/2)
csi_2=50/100
