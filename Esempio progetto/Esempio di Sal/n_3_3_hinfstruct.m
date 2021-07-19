

%% n_3_3_hinfstruct 
% Tuning of the controller(R) with hinfstruct

run main.m
%% NOMINAL DESIGN: nominal values of the uncertain parameters.

G = G.NominalValue;
% discretization of the plant using the sampling time Ts
G = c2d(G,Ts);

%% Weight functions
run n_3_1_design_weight_functions.m

%% definition of controller R:

% parameters to be tuned
b = realp('b',1);
c1 = realp('c1',1);
c2 = realp('c2',1);
d1 = realp('d1',1);
d2 = realp('d2',1);
D_phi = realp('D_phi',1);

% fixed state-space matrices of the controller
A_R=[1 0;0 0];
B_R=[b -b*D_phi; 0 0.5];
C_R=[c1 c2];
D_R=[d1 d2*D_phi];

R0= ss(A_R,B_R,C_R,D_R,Ts);
R0.InputName={'e_phi';'p'}; % input signals of R
R0.OutputName={'delta_{lat}'}; % output signal of R

%% Creation of plant P to tune 

S = sumblk('%e_phi=phi0-%phi',R0.InputName(1),G.OutputName(2));

W1 = c2d(Wp,Ts); % discretization of the performance weigth
W1.InputName='e_phi'; % input signal of W1
W1.OutputName='z1'; % output signal of W1

% plant P: - the inputs of P are phi0(external input) and delta_lat(from R)
%          - the outputs of P are z1 that is the e_phi weigthed with W1
%            (performance requirement), e_phi and p that are the outputs of
%            G.
P = connect(S,G,W1,{'phi0','delta_{lat}'},{'z1';'e_phi';'p'});
P.InputGroup.U1 =[1];
P.InputGroup.U2 =[2];
P.OutputGroup.Y1 = [1];
P.OutputGroup.Y2 = [2,3];

%% hinfstruct
% tuning of R with the performace weight
opt=hinfstructOptions('Display','final','RandomStart',5);
R = hinfstruct(P,R0,opt);