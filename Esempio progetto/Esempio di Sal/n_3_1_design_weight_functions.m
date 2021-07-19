%% n_3_1_design_weight_functions
% evaluate the weigth functions for the hinf tuning

s=tf('s');
%% Weight of the performance

% performace requirements
omega_weight=10; %[rad/s] minimum cross-over frequency
csi_weight=0.9; % minimum damping ratio
M=1;

% Case A
% a=1e-3;
% Wp_inv=(s+a*omega_min)/(s/M+omega_min); % inverse of the perfotmance weight function
% 
% Wp=1/Wp_inv; %evaluation of Wp

% Case B
F_weight=omega_weight^2/(s^2+2*csi_weight*omega_weight*s+omega_weight^2);
S_weight=1-F_weight;
Wp=1/S_weight;
Wp_inv=S_weight;


% plot
opt_bodeWp_inv = bodeoptions;
opt_bodeWp_inv.Title.String = '\textbf{Bode Diagram}';
opt_bodeWp_inv.Title.Interpreter = 'latex';
opt_bodeWp_inv.Title.FontSize = 12;
opt_bodeWp_inv.XLabel.String = '\textbf{Frequency}';
opt_bodeWp_inv.XLabel.Interpreter = 'latex';
opt_bodeWp_inv.XLabel.FontSize = 10;
opt_bodeWp_inv.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_bodeWp_inv.YLabel.Interpreter = 'latex';
opt_bodeWp_inv.YLabel.FontSize = 10;
opt_bodeWp_inv.Grid = 'on';
bode(Wp_inv,opt_bodeWp_inv)
set(findall(gcf,'type','line'),'linewidth',1.1)
legend('1/W_p')

%% Weight of the control effort
Wc = [];

%% Weight of the robust stability
Wrs=[];