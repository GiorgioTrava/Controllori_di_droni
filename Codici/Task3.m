%% MODELLO DA REGOLARE INCERTO E NOMINALE (CON M_DELTA FORM) %%

% clc, clear, close all

% run Task1
% run Task2

%% Pid controller 

R_p = tunablePID('R_p', 'PID');
R_p.InputName = {'p_error'};       
R_p.OutputName = {'delta_{lat}'};

R_phi = tunablePID('R_phi', 'P');
R_phi.InputName = {'phi_error'};       
R_phi.OutputName = {'p_0'};
%% Uncertain to be tuned complete system
Sum_p = sumblk('p_error = p_0 - p');
Sum_phi = sumblk('phi_error = phi_0 - phi');


innerLoop= connect(R_p,SYS,Sum_p,'p_0',{'p','phi'},'delta_{lat}');

outerLoop= connect(R_phi,innerLoop,Sum_phi,'phi_0',{'p','phi'});

figure(6)
pzmap(innerLoop)
grid on, hold on
pzmap(outerLoop)
legend('inner loop poles and zeros', 'outer loop poles and zeros')


figure (7)
bode(innerLoop)
grid on, hold on
bode(outerLoop)
legend('inner loop bode diagram', 'outer loop bode diagram')
%% Nominal to be tuned complete system
innerLoop_n= connect(R_p,SYSn,Sum_p,'p_0',{'p','phi'},'delta_{lat}');

outerLoop_n= connect(R_phi,innerLoop_n,Sum_phi,'phi_0',{'p','phi'});


%% M_delta form to be tuned complete system
[M_outerLoop,Delta_outerLoop] = lftdata(outerLoop);
% figure (8)
% bode(lft(Delta_outerLoop,M_outerLoop))
