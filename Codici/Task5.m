%% VERIFICA RS %%
clc, clear, close all

run Task1
% run Task2
run Task3
run Task4

%% creo modello incerto controllato

%% Pid controller after tuning

R_p_C = getBlockValue(outerLoop_n_C,'R_p')
R_p_C.InputName = {'p_error'};       
R_p_C.OutputName = {'delta_{lat}'};

R_phi_C = getBlockValue(outerLoop_n_C,'R_phi')
R_phi_C.InputName = {'phi_error'};       
R_phi_C.OutputName = {'p_0'};
%% Uncertain to be tuned complete system

innerLoop_C= connect(R_p_C,SYS,Sum_p,'p_0',{'p','phi'},'delta_{lat}');

outerLoop_C= connect(R_phi_C,innerLoop_C,Sum_phi,'phi_0',{'p','phi'});

F_p=outerLoop_C(1)
F_phi=outerLoop_C(2)
%% 
% [M_outerLoop_C,Delta_outerLoop_C] = lftdata(outerLoop);
% outerLoop_C=lft(Delta_outerLoop_C,M_outerLoop_C)
outerLoop_C_p_array=usample(outerLoop_C(1),60);
[U_outerLoop_C_p,Info_p]=ucover(outerLoop_C_p_array,outerLoop_n_C(1),9);

outerLoop_C_phi_array=usample(outerLoop_C(2),60);
[U_outerLoop_C_phi,Info_phi]=ucover(outerLoop_C_phi_array,outerLoop_n_C(2),9);

figure(500)
bodemag(F_p,1/Info_p.W1)
hold on

figure(501)
bodemag(F_phi,1/Info_phi.W1)
hold on
%relative errors
%error=(outerLoop_n_C(2)-outerLoop_C_phi_array)/outerLoop_n_C(2);

