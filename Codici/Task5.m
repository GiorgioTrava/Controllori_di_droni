%% VERIFICA RS %%

% clc, clear, close all

  close all
% run Task1
% run Task2
% run Task3
% run Task4

%% creo modello incerto controllato

outerLoop_C = connect(R_phi_C,R_p_C,SYS,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','delta_{lat}'});
figure(320)
bode(outerLoop_C)
% figure(321)
% nyquist(outerLoop_C)


F_p = outerLoop_C(1);%getIOTransfer(outerLoop_C,'phi_0','phi');%outerLoop_C(1)
F_phi = outerLoop_C(2);
% figure(322)
% bode(F_phi)
F_phi_n = getNominal(F_phi);
F_phi_n.InputName = {'delta_{lat}'};
F_phi_n.OutputName = {'phi_nom'};
S_phi = 1-F_phi;
F_p_n = getNominal(F_p);
F_p_n.InputName = {'delta_{lat}'};
F_p_n.OutputName = {'p_nom'};
S_p = 1-F_p;
%% RS
% [M_outerLoop_C,Delta_outerLoop_C] = lftdata(outerLoop);
% outerLoop_C=lft(Delta_outerLoop_C,M_outerLoop_C)


F_phi_array = usample(F_phi,60);
[U_F_phi,Info_phi] = ucover(F_phi_array,F_phi_n,5);

F_p_array = usample(F_p,60);
[U_F_p,Info_p] = ucover(F_p_array,F_p_n,5);
   % NOTA:  USYS = ucover(PARRAY,PNOM,ORD) uses the simplified uncertainty model 
        %   USYS = PNOM*(I + W*ULTIDYN)
        %   where W is a scalar-valued filter of order ORD. This corresponds to 
        %   ORD1=ORD and ORD2=[] (W1=W and W2=1).
        %  
        %   [USYS,INFO] = ucover(PARRAY,...) also returns a structure INFO with
        %   optimal filter values over a frequency grid. To reuse this information
        %   and quickly try different orders for W1 and W2, use the syntax:
        %      [USYSnew,INFOnew] = ucover(PNOM,INFO,ORD1new,ORD2new)
W_phi = Info_phi.W1;
W_phi.InputName = {'phi_nom'};
W_phi.OutputName = {'z1'};
W_p = Info_p.W1;
W_p.InputName = {'p_nom'};
W_p.OutputName = {'z2'};

figure(500)
subplot(2,1,1)
%errors
error_phi = (tf(F_phi_n)-tf(F_phi_array))/tf(F_phi_n);%usample((outerLoop_n_C(2)-outerLoop_C(2)),60);
bodemag(error_phi,W_phi)
legend
subplot(2,1,2)
error_p = (tf(F_p_n)-tf(F_p_array))/tf(F_p_n);%usample((outerLoop_n_C(2)-outerLoop_C(2)),60);
bodemag(error_p,W_p)
legend

%% ROBUST STABILITY F_nominal<=1/W
figure(501)
subplot(2,1,1)
bodemag(F_phi_n,1/W_phi)
legend('F_phi_n','1/W_phi')
grid on
subplot(2,1,2)
bodemag(F_p_n,1/W_p)
legend('F_p_n','1/W_p')
grid on

%% ROBUST STABILITY max singular value(F)<1/W

W_i = [W_phi 0; 0 W_p];
figure(507)
sigma(tf(outerLoop_C),1/W_i) %nota:è la stessa cosa che fare le due funzioni una per una
hold on
figure(502)
sigma(W_i*tf(outerLoop_C))%dovrebbe essere l'equivalente dell'M-delta form<1
hold on
[sigma_max_no_input_output,freq_peak_no_input_output] = hinfnorm(W_i*tf(outerLoop_C))

%% ROBUST STABILITY max singular value(M)=H_inf_norm(M)<1


%% M_delta form MATLAB  NOTA: commentare le altre versioni di M_delta per vederne una
%%questa versione è corretta ma mette nelle incertezze i singoli parametri
%%di A-->delta è 4x4
[M_outerLoop_C,Delta_outerLoop_C] = lftdata(outerLoop_C);
M_outerLoop_C.InputName = {'w1','w2','w3','w4','phi_0'};
M_outerLoop_C.OutputName = {'z1','z2','z3','z4','p','phi'};

M_tf_outerLoop_C = tf(M_outerLoop_C); 
M_no_input_output = M_tf_outerLoop_C(1:4,1:4); %M_delta form di MATLAB senza considerare input e output del sistema
[sigma_max_no_input_output_M,freq_peak_no_input_output_M] = hinfnorm(M_no_input_output)
[sigma_max_M,freq_peak_M] = hinfnorm(M_outerLoop_C)

figure(502)
sigma(M_outerLoop_C) %confronto
hold on
sigma(M_no_input_output)

%prova mu
omega=logspace(-3,2,500);
bounds_mu=mussv(frd(M_no_input_output,omega),[1 0;1 0;1 0;1 0]);%verifica che cazzo sono sti zeri e uni

figure(502)
sigma(bounds_mu)
hold on
grid
%% M_delta form tentativo 1
%STRUTTURA DA RIFARE!!!!
Sum_phi_n= sumblk('phi = w1 + phi_nom');
Sum_p_n= sumblk('p = w2 + p_nom');
SYSn.InputName = 'delta_{lat}'; 
SYSn.OutputName = {'p_nom','phi_nom'};%nota: cambio i nomi degli output di SYSn !!
M_outerLoop_C=connect(R_phi_C,R_p_C,SYSn(1),SYSn(2),Sum_phi,Sum_p,Sum_phi_n,Sum_p_n,W_phi,W_p,...
    {'phi_0','w1','w2'},{'p','phi','z1','z2'});

Delta_phi=ultidyn('Delta_phi',[1 1],'Bound',1);
Delta_phi.InputName={'z1'};
Delta_phi.OutputName={'w1'};
Delta_p=ultidyn('Delta_p',[1 1],'Bound',1);
Delta_p.InputName={'z2'};
Delta_p.OutputName={'w2'};
Delta_outerLoop_C= connect(Delta_phi,Delta_p,{'z1','z2'},{'w1','w2'});


outerLoop_C_verifica=connect(Delta_outerLoop_C,M_outerLoop_C,'phi_0',{'p','phi'});




figure(320)
hold on
bode(outerLoop_C_verifica)


% [sigma_max,freq_peak]=hinfnorm(M_outerLoop_C)


M_tf_outerLoop_C=tf(M_outerLoop_C) ;
M_no_input_output= M_tf_outerLoop_C(3:4,2:3); 
% [sigma_max,freq_peak]=hinfnorm(M_no_input_output)
figure(502)
sigma(M_outerLoop_C,M_no_input_output)
hold on

sigma_max = getPeakGain(M_outerLoop_C) %dovrebbe essere mu per sistemi stabili
sigma_max_no_input_output = getPeakGain(M_no_input_output)

%prova mu
omega=logspace(-3,2,500);
bounds_mu=mussv(frd(M_no_input_output,omega),[1 0; 1 0]);

figure(502)
sigma(bounds_mu), grid

% figure(667)
% bode(tf(M_outerLoop_C))
%% M_delta form tentativo 2 (sistema con output vettoriali)
R_C=tf(connect(R_p_C,R_phi_C,Sum_phi,Sum_p,{'phi','p'},'delta_{lat}')); %NOTA:perchè funzioni devo togliere phi_0 dagli input?????
W_i=[tf(W_phi) 0; 0 tf(W_p)];
F_w_yn= minreal(1/(eye(2,2)-tf(SYSn)*R_C)*tf(SYSn)*R_C);
F_w_yn.InputName={'w1','w2'};
M_outerLoop_C=minreal(W_i*F_w_yn);
figure(502)
sigma(tf(M_outerLoop_C)) %in qualche modo equivale al mu -->penso che sia perchè con le matrici assumo delta diagonale
hold on
%prova mu
omega=logspace(-3,2,500);
bounds_mu=mussv(frd(M_outerLoop_C,omega),[1 0; 1 0]);

figure(503)
sigma(bounds_mu), grid
%% Robust stability mu_analysys

[gamma,~] = musynperf(M_no_input_output)
% omega=logspace(-3,2,500);
%  bounds=mussv(frd(M_outerLoop_C,omega),);
% % 
% figure(503)
% sigma(bounds), grid
