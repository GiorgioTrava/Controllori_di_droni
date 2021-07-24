%% ANALISI MODELLO QUADROTOR (CON M_DELTA FORM) %%

% clc, clear, close all

% run Task1

%% Zeros and poles map of the uncertain model


figure(1)
pzmap(SYS)
grid on, hold on
pzmap(SYSn)
legend('Uncertain poles and zeros', 'Nominal poles and zeros')
% write the nominal value of pole 1
text(-4.8,-0.4,'(-4.25; 0)')
% write the nominal value of pole 2
text(1.3,3.2,'(1.99; 3.60)')
% write the nominal value of pole 3
text(1.3,-3.2,'(1.99; -3.60)')
% write the nominal value of zero
text(-1,-0.4,'(-0.198; 0)')

%% Bode plots
%SYSarray=usample(SYS,100);

opt_figure1 = bodeoptions;
opt_figure1.Title.String = '\textbf{Bode Diagram}';
opt_figure1.Title.Interpreter = 'latex';
opt_figure1.Title.FontSize = 12;
opt_figure1.XLabel.String = '\textbf{Frequency}';
opt_figure1.XLabel.Interpreter = 'latex';
opt_figure1.XLabel.FontSize = 10;
opt_figure1.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_figure1.YLabel.Interpreter = 'latex';
opt_figure1.YLabel.FontSize = 10;
opt_figure1.Grid = 'on';

figure(2)
bode(SYS(2),'k--',opt_figure1)
grid on, hold on
bode(SYSn(2),'r-',opt_figure1)
legend('Uncertain model Bode diagram', 'Nominal model Bode diagram')

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



poliezeri = zpk(SYS);
