%% 
run main.m
%% tuned controller
Ts = 0.004;
D_phi = 0.170549175453535;
b = 1.620895607321319;
c1 = 0.086200237029275;
c2 = 0.103218330013736;
d1 = 0.194734070587707;
d2 = -2.483324671681038;

Ap=[1 0;0 0];
Bp=[b -b*D_phi; 0 0.5];
Cp=[c1 c2];
Dp=[d1 d2*D_phi];

% R0=Cp*inv(z*eye(2)-Ap)+Dp; 
R= ss(Ap,Bp,Cp,Dp,Ts);
R.InputName={'e_phi';'p'};
R.OutputName={'delta_{lat}'};
R = d2c(R,'tustin'); 

%%

% Sum block
S1 = sumblk('%e_phi = phi0-%phi', R.InputName(1), G.OutputName(2));

% Complementary sensitivity function from phi0 to phi
F_nominal = connect(G.NominalValue,R,S1,S1.InputName(1),{'phi'});
F_uncertain = connect(G,R,S1,S1.InputName(1),{'phi'});

% % bode plot of M: check if is lower that zero
% bode(M(1))

%% Creation of W
F_array=usample(F_uncertain,60);
error=(F_nominal-F_array)/F_nominal;

%Bode plot of the error
figure(1)
%options of bode plot
opt_figure1 = bodeoptions;
opt_figure1.Title.String = '\textbf{Bode Diagram of the error}';
opt_figure1.Title.Interpreter = 'latex';
opt_figure1.Title.FontSize = 12;
opt_figure1.XLabel.String = '\textbf{Frequency}';
opt_figure1.XLabel.Interpreter = 'latex';
opt_figure1.XLabel.FontSize = 10;
opt_figure1.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_figure1.YLabel.Interpreter = 'latex';
opt_figure1.YLabel.FontSize = 10;
opt_figure1.Grid = 'on';
bodeplot(error,{10^-4,10^3},opt_figure1)

%Finding W
[F,Info] = ucover(F_array,F_nominal,1); %we ask for a 1st order uncertainty function
W=Info.W1; %extract W from ucover outputs

%Bode plot of the error and W
figure(2)
opt_figure2 = bodeoptions;
opt_figure2.Title.String = '\textbf{Bode magnitude plot of error and W}';
opt_figure2.Title.Interpreter = 'latex';
opt_figure2.Title.FontSize = 12;
opt_figure2.XLabel.String = '\textbf{Frequency}';
opt_figure2.XLabel.Interpreter = 'latex';
opt_figure2.XLabel.FontSize = 10;
opt_figure2.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_figure2.YLabel.Interpreter = 'latex';
opt_figure2.YLabel.FontSize = 10;
opt_figure2.PhaseVisible = 'off';
opt_figure2.Grid = 'on';
bodeplot(error,W,{10^-4,10^3},opt_figure2)
legend('error','W','Location','best')


figure(3)
opt_figure3 = bodeoptions;
opt_figure3.Title.String = '\textbf{Bode magnitude}';
opt_figure3.Title.Interpreter = 'latex';
opt_figure3.Title.FontSize = 12;
opt_figure3.XLabel.String = '\textbf{Frequency}';
opt_figure3.XLabel.Interpreter = 'latex';
opt_figure3.XLabel.FontSize = 10;
opt_figure3.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_figure3.YLabel.Interpreter = 'latex';
opt_figure3.YLabel.FontSize = 10;
opt_figure3.PhaseVisible = 'off';
opt_figure3.Grid = 'on';
bodeplot(F_nominal,1/W,{10^-4,10^3},opt_figure3)
legend('F nominal', '1/W')

%% Creation of matrix M

G_nominal = G.NominalValue;
G_nominal.InputName = 'delta_lat2';
G_nominal.OutputName = {'p','phi'};

R.InputName = {'e_phi','p'};
R.OutputName = 'delta_lat';

W.InputName = 'delta_lat';
W.OutputName = 'z';

S1 = sumblk('%e_phi = phi0-%phi', R.InputName(1), G.OutputName(2));
S2 = sumblk('%delta_lat2 = %delta_lat + w', G_nominal.InputName, R.OutputName);

M = connect(G_nominal,R,W,S1,S2,'w','z');

%M=minreal(-W*F_nominal);

figure(4)
opt_figure4 = bodeoptions;
opt_figure4.Title.String = '\textbf{Bode magnitude plot of M}';
opt_figure4.Title.Interpreter = 'latex';
opt_figure4.Title.FontSize = 12;
opt_figure4.XLabel.String = '\textbf{Frequency}';
opt_figure4.XLabel.Interpreter = 'latex';
opt_figure4.XLabel.FontSize = 10;
opt_figure4.YLabel.String = {'\textbf{Magnitude}', '\textbf{Phase}'};
opt_figure4.YLabel.Interpreter = 'latex';
opt_figure4.YLabel.FontSize = 10;
opt_figure4.PhaseVisible = 'off';
opt_figure4.Grid = 'on';
bodeplot(M,{10^-4,10^3},opt_figure4)

