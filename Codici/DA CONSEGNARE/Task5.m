%% VERIFICA RS, MU ANALYSIS, RP  %%

% clc, clear, close all

  close all
% run Task1
% run Task2
% run Task3
% run Task4

%% creo modello incerto controllato

outerLoop_C = connect(R_phi_C,R_p_C,SYS,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','delta_{lat}'});

%% robust stability margins

[stabmarg,wcu] = robstab(outerLoop_C)

%A robust stability margin greater than 1 means that the system is stable 
%for all values of its modeled uncertainty. A robust stability margin less
%than 1 means that the system becomes unstable for some values of the uncertain
%elements within their specified ranges

%% OVERBOUND DELL'ERRORE 

SYS_phi_array = usample(SYS(2,1),60);
[U_SYS_phi,Info_phi] = ucover(SYS_phi_array,SYSn(2,1),5);

SYS_p_array = usample(SYS(1,1),60);
[U_SYS_p,Info_p] = ucover(SYS_p_array,SYSn(1,1),5);
   % NOTA:  USYS = ucover(PARRAY,PNOM,ORD) uses the simplified uncertainty model 
        %   USYS = PNOM*(I + W*ULTIDYN)
        %   where W is a scalar-valued filter of order ORD. This corresponds to 
        %   ORD1=ORD and ORD2=[] (W1=W and W2=1).
W_phi = Info_phi.W1;
W_phi.InputName = {'delta_{lat}'};
W_phi.OutputName = {'z1'};
W_p = Info_p.W1;
W_p.InputName = {'delta_{lat}'};
W_p.OutputName = {'z2'};

figure(100)
subplot(2,1,1)
%errors
error_phi = (tf(SYSn(2,1))-tf(SYS_phi_array))/tf(SYSn(2,1));%usample((outerLoop_n_C(2)-outerLoop_C(2)),60);
bodemag(error_phi,W_phi)
grid
legend
title('overbound degli errori')
subplot(2,1,2)
error_p = (tf(SYSn(1,1))-tf(SYS_p_array))/tf(SYSn(1,1));%usample((outerLoop_n_C(2)-outerLoop_C(2)),60);
bodemag(error_p,W_p)
grid
xlim([10^-4 10^2])
legend
title('')

%% M_delta form 

Sum_delta_phi = sumblk('delta_{lat}_{phi} = w1 + delta_{lat}');
Sum_delta_p = sumblk('delta_{lat}_{p} = w2 + delta_{lat}');
G_p = SYSn(1);
G_phi = SYSn(2);
G_p.InputName = 'delta_{lat}_{p}'; 
G_phi.InputName = 'delta_{lat}_{phi}'; 
M_outerLoop_C = connect(R_phi_C,R_p_C,G_p,G_phi,Sum_phi,Sum_p,Sum_delta_phi,Sum_delta_p,W_phi,W_p,...
    {'phi_0','w1','w2'},{'p','phi','z1','z2'});
M_no_input_output = connect(R_phi_C,R_p_C,G_p,G_phi,Sum_phi,Sum_p,Sum_delta_phi,Sum_delta_p,W_phi,W_p,...
    {'w1','w2'},{'z1','z2'});

%verfica ricostruendo il modello
Delta_outerLoop_C = ultidyn('Delta_outerLoop_C',[2 2],'Bound',1);
Delta_outerLoop_C.InputName = {'z1','z2'};
Delta_outerLoop_C.OutputName = {'w1','w2'};
outerLoop_C_verifica = connect(Delta_outerLoop_C,M_outerLoop_C,'phi_0',{'p','phi'});

figure(101)
hold on
bode(outerLoop_C_verifica,outerLoop_C)
grid
legend('modello incerto in M-delta form','modello incerto iniziale')
%% RS sigma 
figure(102)
% sigma(M_outerLoop_C)
hold on
sigma(M_no_input_output)


% sigma_max = getPeakGain(M_outerLoop_C) 
sigma_max_no_input_output = getPeakGain(M_no_input_output)


%% mu analysys
omega = logspace(-3,2,500);
bounds_mu = mussv(frd(M_no_input_output,omega),[1 0;1 0]);

figure(102)
sigma(bounds_mu(1)), grid
legend('sigma M','mu')
%% Robust Performances

W_i = [tf(W_p) 0; 0 tf(W_phi)];
Wperf = 1/W_perf_inv; %performance
figure(); bode(Wperf)

figure(104)
sigma((Wperf*ss(S_n_C)),(W_i*outerLoop_n_C))
grid
legend('Wp*S','W*F')
hinfnorm((Wperf*S_n_C))+hinfnorm((W_i*outerLoop_n_C))

% figure
% bodemag((Wperf*ss(S_n_C)),(W_i*outerLoop_n_C))

