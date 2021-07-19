%% ANALISI MODELLO QUADROTOR (CON M_DELTA FORM) %%

% clc, clear, close all

% run Task1

%% Zeros and poles map of the uncertain model


figure(1)
pzmap(SYS)
grid on, hold on
pzmap(SYSn)
legend('Uncertain poles and zeros', 'Nominal poles and zeros')

%% Bode plots
%SYSarray=usample(SYS,100);

figure (2)
bode(SYS)
grid on, hold on
bode(SYSn)
legend('Uncertain model Bode diagram', 'Nominalmodel Bode diagram')

%% Nyquist plot
figure (3)
nyquist(SYS)
grid on, hold on
nyquist(SYSn)
legend('Uncertain model Nyquist diagram', 'Nominalmodel Nyquist diagram')

%% step response
figure (4)
subplot(2,1,1)
stepplot(SYS(1))
axis([0 10 -1000 1000]);     

subplot(2,1,2)
stepplot(SYS(2))
axis([0 10 -1000 1000]); 



%% M_delta form of the plant
[M_SYS,Delta_SYS] = lftdata(SYS);

figure (5)
bode(lft(Delta_SYS,M_SYS))
%norm(usample(SYS-lft(Delta,M),10),'inf')
% %% Frequency response function
% 
% SYSresponse = ufrd(SYS,logspace(-2,2,100))  
% 
% figure (3)
% bode(SYSresponse)
% grid on, hold on
% bode(G)
% legend('Uncertain model Bode diagram', 'Nominalmodel Bode diagram')
%% Ucover della plant


poliezeri = zpk(SYS);
