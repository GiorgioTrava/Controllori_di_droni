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