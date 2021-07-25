%% MODELLO DA REGOLARE INCERTO E NOMINALE + REQUISITI %%

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

% innerLoop= connect(R_p,SYS,Sum_p,'p_0',{'p','phi'}),'delta_{lat}';
% 
% outerLoop= connect(R_phi,innerLoop,Sum_phi,'phi_0',{'p','phi'},);

outerLoop = connect(R_p,R_phi,SYS,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','p_0','delta_{lat}','phi'});

%% Nominal to be tuned complete system
outerLoop_n = getNominal(outerLoop);

%% Requisiti nominali su phi_0 phi
s=tf('s');
omega_n = 10;
ksi = 0.9;
F_required = omega_n^2/(s^2+2*ksi*omega_n*s+omega_n^2);

S_required = 1 - F_required;

hinfnorm(S_required)

L_required = F_required/(1-F_required);

step_required = stepinfo(F_required)

figure(9)
bode(F_required,S_required,L_required)
legend
title('nominal sensitivities')


