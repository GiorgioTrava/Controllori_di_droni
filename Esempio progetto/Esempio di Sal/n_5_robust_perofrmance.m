% clear,clc,close all
% 
run n_4_robust_analysis
run n_3_1_design_weight_functions

S_nominal=1-F_nominal;

%%
figure(1)
opt_bodeRS = bodeoptions;
opt_bodeRS.Title.String = '\textbf{Bode Diagram}';
opt_bodeRS.Title.Interpreter = 'latex';
opt_bodeRS.Title.FontSize = 12;
opt_bodeRS.XLabel.String = '\textbf{Frequency}';
opt_bodeRS.XLabel.Interpreter = 'latex';
opt_bodeRS.XLabel.FontSize = 10;
opt_bodeRS.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_bodeRS.YLabel.Interpreter = 'latex';
opt_bodeRS.YLabel.FontSize = 10;
opt_bodeRS.Grid = 'on';

bode(Wp*S_nominal + W*F_nominal,tf(1),{10^-4,10^3},opt_bodeRS)
set(findall(gcf,'type','line'),'linewidth',1.1)
legend('Wp*S+W*F','Line at 0 dB')
