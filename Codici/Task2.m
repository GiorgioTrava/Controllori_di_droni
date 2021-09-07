%% AEROSPACE CONTROL SYSTEMS %%

%% Zeros and poles map of the uncertain model

P = pole(G);
Z = tzero(G);

figure(1) %plot di poli e zeri del sistema incerto e nominale
pzmap(SYS)
grid on, hold on
pzmap(G)
legend('Uncertain poles and zeros', 'Nominal poles and zeros')

%% Bode plots

figure (2) %diagramma di bode del sistema incerto e nominale
bode(SYS)
grid on, hold on
bode(G)
legend('Uncertain model Bode diagram', 'Nominalmodel Bode diagram')

%% Nyquist plot

figure (3) %diagramma di Nyquist sistema incerto e nominale
nyquist(SYS)
grid on, hold on
nyquist(G)
legend('Uncertain model Bode diagram', 'Nominalmodel Bode diagram')



%poliezeri = zpk(SYS)
