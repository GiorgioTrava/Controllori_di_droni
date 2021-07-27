%% ANALISI MODELLO QUADROTOR  %%

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

%% Singular values e H_inf norm
figure(2)
sigma(SYSn)
grid
hinfnorm(SYSn) 
%% impulse and step response solo SYSn
figure (3)
subplot(2,1,1)
impulseplot(SYS(1))
axis([0 10 -2000 2000]);     

subplot(2,1,2)
impulseplot(SYS(2))
axis([0 10 -1000 1000]); 

figure (4)
subplot(2,1,1)
stepplot(SYS(1))
axis([0 10 -1000 1000]);     

subplot(2,1,2)
stepplot(SYS(2))
axis([0 10 -1000 1000]); 
%% Bode plots

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

figure(5)
bode(SYS,'y--',opt_figure1)
grid on, hold on
bode(SYSn,'r-',opt_figure1)
legend('Uncertain model Bode diagram', 'Nominal model Bode diagram')
hinfnorm(SYSn)

%% Nyquist plot
figure (6)
grid on
nyquist(SYSn)
legend('Nominal model Nyquist diagram') 

%% impulse and step response SYSn con feedback
SYSn_closed = [minreal(SYSn(1)/(1+SYSn(1))); minreal(SYSn(2)/(1+SYSn(2)))];

figure (7)
subplot(2,1,1)
impulseplot(SYSn_closed(1))
% axis([0 10 -2000 2000]);     

subplot(2,1,2)
impulseplot(SYSn_closed(2))
% axis([0 10 -1000 1000]); 

figure (8)
subplot(2,1,1)
stepplot(SYSn_closed(1,1))
% axis([0 10 -1000 1000]);     

subplot(2,1,2)
stepplot(SYSn_closed(2,1))
% axis([0 10 -1000 1000]); 


 

