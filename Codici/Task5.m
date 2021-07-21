%% VERIFICA RS %%
% clc, clear, close all
  close all
% run Task1
% % run Task2
% run Task3
% run Task4

%% creo modello incerto controllato

outerLoop_C= connect(R_phi_C,R_p_C,SYS,Sum_phi,Sum_p,'phi_0',{'p','phi'},{'phi_error','delta_{lat}'});
figure(320)
bode(outerLoop_C)
figure(321)
nyquist(outerLoop_C)


F_p=outerLoop_C(1);%getIOTransfer(outerLoop_C,'phi_0','phi');%outerLoop_C(1)
F_phi=outerLoop_C(2);
% figure(322)
% bode(F_phi)
F_phi_n=getNominal(F_phi);
S_phi=1-F_phi;
%% RS
% [M_outerLoop_C,Delta_outerLoop_C] = lftdata(outerLoop);
% outerLoop_C=lft(Delta_outerLoop_C,M_outerLoop_C)


F_phi_array=usample(F_phi,60);
[U_F_phi,Info_phi]=ucover(F_phi_array,F_phi_n,3);
   % NOTA:  USYS = ucover(PARRAY,PNOM,ORD) uses the simplified uncertainty model 
        %   USYS = PNOM*(I + W*ULTIDYN)
        %   where W is a scalar-valued filter of order ORD. This corresponds to 
        %   ORD1=ORD and ORD2=[] (W1=W and W2=1).
        %  
        %   [USYS,INFO] = ucover(PARRAY,...) also returns a structure INFO with
        %   optimal filter values over a frequency grid. To reuse this information
        %   and quickly try different orders for W1 and W2, use the syntax:
        %      [USYSnew,INFOnew] = ucover(PNOM,INFO,ORD1new,ORD2new)
W=Info_phi.W1;

figure(502)
%errors
error_phi=abs(F_phi_n-F_phi_array)/abs(F_phi_n);%usample((outerLoop_n_C(2)-outerLoop_C(2)),60);
bodemag(error_phi,W)
legend

%ROBUST STABILITY F_nominal<=1/W
figure(501)
bodemag(F_phi_n,1/W)
legend('F_{nomninal}','1/W')
grid on

%ROBUST STABILITY max singular value(M)=H_inf_norm(M)<1

[M_outerLoop_C,Delta_outerLoop_C] = lftdata(outerLoop_C);

[sigma_max,freq_peak]=hinfnorm(M_outerLoop_C)
sigma(M_outerLoop_C)


