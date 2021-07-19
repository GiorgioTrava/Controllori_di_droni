close all
%% n_3_2_final_plots
% shows the results obtained with hinf

% need to run n_3_3_hinfsctruct.m to work:
% run n_3_3_hinfsctruct.m
%% Creation of the tuned system

% Summing junctions
S1 = sumblk('%e_phi = phi0-%phi', R0.InputName(1), G.OutputName(2));

% complementary sensitivity function from phi0 to phi
F = connect(G,R,S1,S1.InputName(1),G.OutputName(2));
% sensitivity function ---> F+S=1 
S = 1-F;
% control sensitivity function
Q = connect(G,R,S1,S1.InputName(1),R.OutputName);

%% complementary sensitivity function plots
omega_min=10;
csi=0.9;
% F2 is a reference second order function with minimum requirements of 
% cross-over frequency and damping ratio.
F2=omega_min^2/(s^2+2*csi*omega_min*s+omega_min^2);

figure(1) % step response comparison of F and F2
stepplot(F,F2),legend('F','F2'), grid on
title('\textbf{Step Response}','Interpreter','latex')
xlabel('Time','Interpreter','latex')
ylabel('Amplitude','Interpreter','latex')

figure(2) % frequency response function: F vs F2
% definition of all the options for the plot
opt_bodeF = bodeoptions;
opt_bodeF.Title.String = '\textbf{Bode Diagram}';
opt_bodeF.Title.Interpreter = 'latex';
opt_bodeF.Title.FontSize = 12;
opt_bodeF.XLabel.String = '\textbf{Frequency}';
opt_bodeF.XLabel.Interpreter = 'latex';
opt_bodeF.XLabel.FontSize = 10;
opt_bodeF.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_bodeF.YLabel.Interpreter = 'latex';
opt_bodeF.YLabel.FontSize = 10;
opt_bodeF.Grid = 'on';

bode(F,F2,opt_bodeF)
set(findall(gcf,'type','line'),'linewidth',1.1)
legend('F','F2')
%% control sensitivity function plots 
Q=d2c(tf(Q),'tustin')
figure(3)
opt_bodeQ = opt_bodeF; % use the same options 
bode(Q,opt_bodeQ)
legend('Q')
set(findall(gcf,'type','line'),'linewidth',1.1)

figure(4)
opt_step = stepDataOptions('StepAmplitude',10);
step(Q,opt_step)
title1=title('\textbf{Step Response}','Interpreter','latex')
set(title1,'FontSize',20);
xlabel('Time','Interpreter','latex')
ylabel('Amplitude','Interpreter','latex')

%% sensitivity function plots 
figure(5) % sensitivity function vs performance weigth
opt_bodeS = opt_bodeF; % use the same options 
bode(S,Wp_inv,opt_bodeS)
legend('S','1/Wp')
set(findall(gcf,'type','line'),'linewidth',1.1)

%% F_weight and S_weight
figure(6)
bode(F_weight,S_weight);

%% Evaluation of damping factor
L=connect(R,G,{'e_phi'},{'phi'});
[~,Pm]=margin(F);
csi_1=abs(Pm)/100;
csi_2=sin(abs(deg2rad(Pm))/2);

