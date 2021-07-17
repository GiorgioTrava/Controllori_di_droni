%% SCELTA GUADAGNI E VERIFICA %%
clc, clear, close all

run Task1
run Task2
run Task3

%% requisiti NP: funzione di trasferimento tra phi e phi_0
omega_n_2 = 10;
epsilon_2 = 0.9;
numeratore_2 = omega_n_2^2;
denominatore_2 = [1, 2*omega_n_2*epsilon_2, omega_n_2^2];
G_required_2 = t_f(numeratore_2, denominatore_2)

omega_n = 15;
epsilon = 0.98;
numeratore = omega_n^2;
denominatore = [1, 2*omega_n*epsilon, omega_n^2];
G_required = t_f(numeratore, denominatore)

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



%% verifica control effort per doublet di phi_0 %NOTA: da fare dopo il tuning!!!!

t_f=20;
timestep=0.001;
t=[0:timestep:t_f];
doublet_phi_0=interp1([0,1-timestep,1,3-timestep,3,5-timestep,5,t_f],[0,0,10,10,-10,-10,0,0],t);%deg!!!!!
figure(8)
plot(t,doublet_phi_0,'-')

%funzione di trasferimento phi_0 delta_lat
H1 = getIOTransfer(outerLoop_n,'phi_0','delta_{lat}')
figure(9)
lsim(H1,doublet_phi_0*pi/180,t)

