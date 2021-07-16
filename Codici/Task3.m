%% AEROSPACE CONTROL SYSTEMS %%

clc, clear, close all

run Task1
run Task2

%% Pid controller

R_p = tunablePID('PIDcontroler', 'PID');
R_p.InputName = {'p_error'}       
R_p.OutputName = {'delta_lat'}

R_phi = tunablePID('Pcontroler', 'P');
R_phi.InputName = {'phi_error'}       
R_phi.OutputName = {'p_0'}

X = AnalysisPoint('X');
innerLoop = feedback(R_p * G,1,1,1,-1)

outerLoop = feedback(R_phi * innerLoop,X,1,2,-1)
outerLoop.InputName = 'phi_0';
outerLoop.OutputName = {'p','phi'};

figure()
pzmap(outerLoop)

tzero(outerLoop)
pole(outerLoop)

%% requisiti NP: funzione di trasferimento tra phi e phi_0
omega_n_2 = 10;
epsilon_2 = 0.9;
numeratore_2 = omega_n_2^2;
denominatore_2 = [1, 2*omega_n_2*epsilon_2, omega_n_2^2];
G_required_2 = tf(numeratore_2, denominatore_2)

omega_n = 15;
epsilon = 0.98;
numeratore = omega_n^2;
denominatore = [1, 2*omega_n*epsilon, omega_n^2];
G_required = tf(numeratore, denominatore)

figure(100) 
bode(G_required)
hold on
bode(G_required_2, 'r')

Req1 = TuningGoal.Transient('phi_0','phi',G_required, 'step');
Req2 = TuningGoal.Transient('phi_0','phi',G_required_2, 'step');

[CL,fSoft,fHard] = systune(outerLoop,Req1, Req2);
hold on
bode(CL(2))
hold on
%viewGoal(Req,CL)
figure()
pzmap(outerLoop)
hold on
pzmap(CL)

figure(101)
step(G_required)
hold on
step(G_required_2)
step(CL(2))
