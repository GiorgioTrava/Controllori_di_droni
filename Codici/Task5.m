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

% innerLoop_C= connect(R_p_C,SYS,Sum_p,'p_0',{'p','phi'},'delta_{lat}');
% 
% outerLoop_C= connect(R_phi_C,innerLoop_C,Sum_phi,'phi_0',{'p','phi'});
outerLoop_C= connect(R_phi_C,R_p_C,SYS,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','delta_{lat}'});
figure(320)
bode(outerLoop_C)

F_p=outerLoop_C(1);%getIOTransfer(outerLoop_C,'phi_0','phi');%outerLoop_C(1)
F_phi=outerLoop_C(2)
%% 
% [M_outerLoop_C,Delta_outerLoop_C] = lftdata(outerLoop);
% outerLoop_C=lft(Delta_outerLoop_C,M_outerLoop_C)


outerLoop_C_phi_array=usample(outerLoop_C(2),60);
[U_outerLoop_C_phi,Info_phi]=ucover(outerLoop_C_phi_array,outerLoop_n_C(2),1);


figure(501)
bodemag(F_phi,1/Info_phi.W1)
hold on



% figure(502)
% %relative errors
% error_phi=(outerLoop_n_C(2)-outerLoop_C_phi_array);%usample((outerLoop_n_C(2)-outerLoop_C(2)),60);
% bodemag(error_phi,Info_phi.W1)
