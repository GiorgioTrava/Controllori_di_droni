%% AEROSPACE CONTROL SYSTEMS %%

clc, clear, close all

run Task1

%% Zeros and poles map of the uncertain model

P = pole(G);
Z = tzero(G);

figure(1)
pzmap(SYS)
grid on, hold on
pzmap(G)
legend('Uncertain poles and zeros', 'Nominal poles and zeros')

%% Bode plots

figure (2)
bode(SYS)
grid on, hold on
bode(G)
legend('Uncertain model Bode diagram', 'Nominalmodel Bode diagram')

%% Nyquist plot
figure (3)
nyquist(SYS)
grid on, hold on
nyquist(G)
legend('Uncertain model Bode diagram', 'Nominalmodel Bode diagram')
% %% Frequency response function
% 
% SYSresponse = ufrd(SYS,logspace(-2,2,100))  
% 
% figure (3)
% bode(SYSresponse)
% grid on, hold on
% bode(G)
% legend('Uncertain model Bode diagram', 'Nominalmodel Bode diagram')

poliezeri = zpk(SYS)
