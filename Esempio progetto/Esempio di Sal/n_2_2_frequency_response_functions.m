%% n2_2_frequency_response_functions.m 
% Figure 1 shows the nominal and uncertain TF from delta_lat to output p
% Figure 2 shows the nominal and uncertain TF from delta_lat to output phi

run n_1_main.m
%% Bode plots

% from delta_lat to p
figure(1)
% definition of all the options for the plot
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

% uncertain plot
bode(G(1),'y--',opt_figure1)
hold on  
% nominal plot
bode(G.NominalValue(1),'k-',opt_figure1)
set(findall(gcf,'type','line'),'linewidth',1.5)
legend('Uncertain', 'Nominal')


% from delta_lat to phi
figure(2)
% definition of all the options for the plot
opt_figure2 = bodeoptions;
opt_figure2.Title.String = '\textbf{Bode Diagram}';
opt_figure2.Title.Interpreter = 'latex';
opt_figure2.Title.FontSize = 12;
opt_figure2.XLabel.String = '\textbf{Frequency}';
opt_figure2.XLabel.Interpreter = 'latex';
opt_figure2.XLabel.FontSize = 10;
opt_figure2.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_figure2.YLabel.Interpreter = 'latex';
opt_figure2.YLabel.FontSize = 10;
opt_figure1.Grid = 'on';

% uncertain plot
bode(G(2),'y--',opt_figure1)
hold on 
% nominal plot
bode(G.NominalValue(2),'k-',opt_figure1)
set(findall(gcf,'type','line'),'linewidth',1.5)
legend('Uncertain', 'Nominal')
